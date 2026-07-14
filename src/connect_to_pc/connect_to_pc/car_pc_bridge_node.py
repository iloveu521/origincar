#!/usr/bin/env python3
"""ROS2 edge node that transports camera frames to a PC VLM over HTTP."""

import socket
import threading
import time
from enum import Enum
from typing import Callable, Optional, Union

import cv2
from cv_bridge import CvBridge
from flask import Flask, jsonify, request
import rclpy
from rclpy.node import Node
import requests
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Empty, String
from werkzeug.serving import make_server


def _normalize_path(path: str) -> str:
    if not path:
        return '/'
    return path if path.startswith('/') else f'/{path}'


class BridgeState(Enum):
    """Single-shot bridge states."""

    WAITING_FOR_IMAGE = 'WAITING_FOR_IMAGE'
    SENDING = 'SENDING'
    SUCCEEDED = 'SUCCEEDED'


class CallbackHttpServer:
    """Small HTTP callback server used by the PC service to push results."""

    def __init__(
        self,
        host: str,
        port: int,
        path: str,
        on_description: Callable[[str, str], None],
        logger,
    ) -> None:
        self._host = host
        self._port = port
        self._path = _normalize_path(path)
        self._on_description = on_description
        self._logger = logger
        self._app = Flask(__name__)
        self._server = None
        self._thread: Optional[threading.Thread] = None
        self._register_routes()

    def _register_routes(self) -> None:
        @self._app.route(self._path, methods=['POST'])
        def receive_result():
            data = request.get_json(silent=True) or {}
            description = str(data.get('description', '')).strip()
            if not description:
                return jsonify({
                    'status': 'error',
                    'message': 'missing description',
                }), 400

            self._on_description(description, 'callback')
            return jsonify({'status': 'ok'})

    def start(self) -> bool:
        probe_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            probe_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe_socket.bind((self._host, self._port))
        except OSError as exc:
            self._logger.warn(
                f'Callback receiver disabled: failed to listen on '
                f'{self._host}:{self._port}{self._path}: {exc}. '
                'The node will still publish descriptions from the '
                'PC /predict HTTP response.')
            return False
        finally:
            probe_socket.close()

        try:
            self._server = make_server(
                self._host,
                self._port,
                self._app,
                threaded=True,
            )
        except (OSError, SystemExit) as exc:
            self._logger.warn(
                f'Callback receiver disabled: failed to listen on '
                f'{self._host}:{self._port}{self._path}: {exc}. '
                'The node will still publish descriptions from the '
                'PC /predict HTTP response.')
            return False

        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name='car_pc_callback_http_server',
            daemon=True,
        )
        self._thread.start()
        self._logger.info(
            f'Callback receiver listening on {self._host}:{self._port}{self._path}')
        return True

    def shutdown(self) -> None:
        if self._server is not None:
            self._server.shutdown()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)


class CarPcBridgeNode(Node):
    """Sends throttled camera frames to a PC server and republishes text."""

    def __init__(self) -> None:
        super().__init__('car_pc_bridge')

        self._declare_parameters()
        self._bridge = CvBridge()
        self._pending_lock = threading.Lock()
        self._result_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._send_event = threading.Event()
        self._stop_event = threading.Event()

        self._pending_image: Optional[Union[CompressedImage, Image]] = None
        self._send_inflight = False
        self._next_send_time = 0.0
        self._last_result = ''
        self._last_result_time = 0.0
        self._state = BridgeState.WAITING_FOR_IMAGE
        self._last_state_name = ''
        self._capture_requested = False

        self._description_pub = self.create_publisher(
            String,
            self.get_parameter('description_topic').value,
            10,
        )
        self._state_pub = None
        if self.get_parameter('publish_state').value:
            self._state_pub = self.create_publisher(
                String,
                self.get_parameter('state_topic').value,
                10,
            )
        image_topic = self.get_parameter('image_topic').value
        image_msg_type = self.get_parameter('image_msg_type').value
        if image_msg_type == 'raw':
            self._image_sub = self.create_subscription(
                Image,
                image_topic,
                self._image_callback,
                10,
            )
        else:
            self._image_sub = self.create_subscription(
                CompressedImage,
                image_topic,
                self._image_callback,
                10,
            )
        self._trigger_sub = None
        if self.get_parameter('trigger_required').value:
            self._trigger_sub = self.create_subscription(
                Empty,
                self.get_parameter('trigger_topic').value,
                self._trigger_callback,
                10,
            )

        self._car_ip = self._resolve_car_ip()
        self._callback_server: Optional[CallbackHttpServer] = None
        if self.get_parameter('enable_callback_receiver').value:
            self._callback_server = CallbackHttpServer(
                self.get_parameter('callback_host').value,
                int(self.get_parameter('callback_port').value),
                self.get_parameter('callback_path').value,
                self._publish_description,
                self.get_logger(),
            )
            if not self._callback_server.start():
                self._callback_server = None

        self._sender_thread = threading.Thread(
            target=self._sender_loop,
            name='car_pc_http_sender',
            daemon=True,
        )
        self._sender_thread.start()

        self.get_logger().info(
            'Car-PC HTTP bridge started. '
            f'image_topic={self.get_parameter("image_topic").value}, '
            f'image_msg_type={self.get_parameter("image_msg_type").value}, '
            f'description_topic={self.get_parameter("description_topic").value}, '
            f'pc_server_url={self.get_parameter("pc_server_url").value}, '
            f'car_ip={self._car_ip}, '
            f'trigger_required={self.get_parameter("trigger_required").value}')
        self._set_state(BridgeState.WAITING_FOR_IMAGE)

    def _declare_parameters(self) -> None:
        self.declare_parameter('image_topic', '/image')
        self.declare_parameter('image_msg_type', 'compressed')
        self.declare_parameter('description_topic', '/image_description')
        self.declare_parameter('state_topic', '/image_description_state')
        self.declare_parameter('trigger_required', True)
        self.declare_parameter('trigger_topic', '/capture_trigger')
        self.declare_parameter('pc_server_url', 'http://192.168.3.12:9999/predict')
        self.declare_parameter('request_timeout_sec', 20.0)
        self.declare_parameter('min_send_period_sec', 0.3)
        self.declare_parameter('retry_backoff_sec', 5.0)
        self.declare_parameter('resize_width', 320)
        self.declare_parameter('resize_height', 320)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('result_json_key', 'description')
        self.declare_parameter('publish_max_chars', 200)
        self.declare_parameter('dedup_window_sec', 3.0)
        self.declare_parameter('publish_response_result', True)
        self.declare_parameter('stop_after_first_result', True)
        self.declare_parameter('publish_state', True)
        self.declare_parameter('enable_callback_receiver', True)
        self.declare_parameter('callback_host', '0.0.0.0')
        self.declare_parameter('callback_port', 8888)
        self.declare_parameter('callback_path', '/receive_result')
        self.declare_parameter('car_ip', '')
        self.declare_parameter('ip_probe_host', '192.168.3.12')
        self.declare_parameter('ip_probe_port', 9999)

    def _resolve_car_ip(self) -> str:
        configured_ip = str(self.get_parameter('car_ip').value).strip()
        if configured_ip:
            return configured_ip

        probe_host = str(self.get_parameter('ip_probe_host').value)
        probe_port = int(self.get_parameter('ip_probe_port').value)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.connect((probe_host, probe_port))
            return sock.getsockname()[0]
        except OSError as exc:
            self.get_logger().warn(
                f'Failed to auto-detect car IP via {probe_host}:{probe_port}: {exc}. '
                'Fallback to 127.0.0.1.')
            return '127.0.0.1'
        finally:
            sock.close()

    def _image_callback(self, msg: Union[CompressedImage, Image]) -> None:
        if self._should_stop_after_result():
            return

        now = time.monotonic()
        with self._pending_lock:
            if self._should_stop_after_result():
                return
            if (
                self.get_parameter('trigger_required').value
                and not self._capture_requested
            ):
                return
            if self._send_inflight or now < self._next_send_time:
                return

            self._pending_image = msg
            self._send_inflight = True
            if self.get_parameter('trigger_required').value:
                self._capture_requested = False
            self._next_send_time = (
                now + float(self.get_parameter('min_send_period_sec').value))
        self._set_state(BridgeState.SENDING)
        self._send_event.set()

    def _trigger_callback(self, msg: Empty) -> None:
        del msg
        if self._should_stop_after_result():
            return

        with self._pending_lock:
            self._capture_requested = True
            self._next_send_time = 0.0

        self.get_logger().info('Capture trigger received; waiting for next image')
        self._set_state(BridgeState.WAITING_FOR_IMAGE)

    def _sender_loop(self) -> None:
        while not self._stop_event.is_set():
            if not self._send_event.wait(timeout=0.2):
                continue
            self._send_event.clear()

            with self._pending_lock:
                image_msg = self._pending_image
                self._pending_image = None

            if image_msg is None:
                self._mark_send_complete(False)
                if not self._should_stop_after_result():
                    self._set_state(BridgeState.WAITING_FOR_IMAGE)
                continue

            if self._should_stop_after_result():
                self._mark_send_complete(True)
                continue

            success = self._send_image(image_msg)
            self._mark_send_complete(success)
            if not self._should_stop_after_result():
                self._set_state(BridgeState.WAITING_FOR_IMAGE)

    def _mark_send_complete(self, success: bool) -> None:
        with self._pending_lock:
            self._send_inflight = False
            if self._should_stop_after_result():
                self._pending_image = None
                return
            if not success:
                self._next_send_time = (
                    time.monotonic()
                    + float(self.get_parameter('retry_backoff_sec').value)
                )
                if self.get_parameter('trigger_required').value:
                    self._capture_requested = True

    def _send_image(self, image_msg: Union[CompressedImage, Image]) -> bool:
        if self._should_stop_after_result():
            return True

        try:
            jpeg_bytes = self._encode_image(image_msg)
        except Exception as exc:
            self.get_logger().error(f'Image encode failed: {exc}')
            return False

        files = {'image': ('image.jpg', jpeg_bytes, 'image/jpeg')}
        params = {'car_ip': self._car_ip}
        timeout = float(self.get_parameter('request_timeout_sec').value)

        try:
            response = requests.post(
                self.get_parameter('pc_server_url').value,
                params=params,
                files=files,
                timeout=timeout,
            )
            if response.status_code != 200:
                self.get_logger().warn(
                    f'PC server returned HTTP {response.status_code}: '
                    f'{response.text[:120]}')
                return False

            if self.get_parameter('publish_response_result').value:
                result_key = self.get_parameter('result_json_key').value
                data = response.json()
                description = str(data.get(result_key, '')).strip()
                if description:
                    self._publish_description(description, 'response')

            return True
        except requests.exceptions.Timeout:
            self.get_logger().error('PC server request timed out')
        except requests.exceptions.ConnectionError as exc:
            self.get_logger().error(f'Cannot connect to PC server: {exc}')
        except ValueError as exc:
            self.get_logger().error(f'Invalid JSON response from PC server: {exc}')
        except requests.RequestException as exc:
            self.get_logger().error(f'HTTP request failed: {exc}')
        return False

    def _encode_image(self, image_msg: Union[CompressedImage, Image]) -> bytes:
        if isinstance(image_msg, CompressedImage):
            return bytes(image_msg.data)

        cv_image = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        width = int(self.get_parameter('resize_width').value)
        height = int(self.get_parameter('resize_height').value)
        quality = int(self.get_parameter('jpeg_quality').value)

        if width > 0 and height > 0:
            cv_image = cv2.resize(
                cv_image,
                (width, height),
                interpolation=cv2.INTER_AREA,
            )

        ok, encoded = cv2.imencode(
            '.jpg',
            cv_image,
            [cv2.IMWRITE_JPEG_QUALITY, quality],
        )
        if not ok:
            raise RuntimeError('cv2.imencode returned false')
        return encoded.tobytes()

    def _publish_description(self, description: str, source: str) -> None:
        max_chars = int(self.get_parameter('publish_max_chars').value)
        text = description.strip()
        if not text:
            return
        if max_chars > 0:
            text = text[:max_chars]

        now = time.monotonic()
        dedup_window = float(self.get_parameter('dedup_window_sec').value)
        stop_after_first = self.get_parameter('stop_after_first_result').value
        publish_succeeded_state = False
        with self._result_lock:
            if stop_after_first:
                with self._state_lock:
                    if self._state == BridgeState.SUCCEEDED:
                        self.get_logger().debug(
                            f'Ignored result from {source}; bridge succeeded')
                        return
                    self._state = BridgeState.SUCCEEDED
                    self._last_state_name = BridgeState.SUCCEEDED.value
                    publish_succeeded_state = True

            if (
                text == self._last_result
                and now - self._last_result_time < dedup_window
            ):
                self.get_logger().debug(f'Dropped duplicate result from {source}')
                return
            self._last_result = text
            self._last_result_time = now

        msg = String()
        msg.data = text
        self._description_pub.publish(msg)
        self.get_logger().info(f'Published image description from {source}: {text}')

        if stop_after_first:
            with self._pending_lock:
                self._pending_image = None
                self._next_send_time = float('inf')
            if publish_succeeded_state:
                self.get_logger().info(
                    f'Car-PC bridge state: {BridgeState.SUCCEEDED.value}')
                if self._state_pub is not None:
                    state_msg = String()
                    state_msg.data = BridgeState.SUCCEEDED.value
                    self._state_pub.publish(state_msg)

    def _should_stop_after_result(self) -> bool:
        if not self.get_parameter('stop_after_first_result').value:
            return False
        with self._state_lock:
            return self._state == BridgeState.SUCCEEDED

    def _set_state(self, state: BridgeState) -> None:
        state_name = state.value
        with self._state_lock:
            if (
                self.get_parameter('stop_after_first_result').value
                and self._state == BridgeState.SUCCEEDED
                and state != BridgeState.SUCCEEDED
            ):
                return
            changed = self._state != state
            self._state = state
            if not changed and self._last_state_name == state_name:
                return
            self._last_state_name = state_name

        self.get_logger().info(f'Car-PC bridge state: {state_name}')
        if self._state_pub is not None:
            msg = String()
            msg.data = state_name
            self._state_pub.publish(msg)

    def destroy_node(self) -> None:
        self._stop_event.set()
        self._send_event.set()
        if hasattr(self, '_sender_thread') and self._sender_thread.is_alive():
            self._sender_thread.join(timeout=2.0)
        if self._callback_server is not None:
            self._callback_server.shutdown()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CarPcBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
