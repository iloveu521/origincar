"""Topic-based speech broadcast manager."""

import heapq
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .speech_client import SpeechClient, SpeechClientError


@dataclass(order=True)
class BroadcastRequest:
    """Queued speech request ordered by priority and insertion sequence."""

    priority_key: int
    sequence: int
    source: str = field(compare=False)
    text: str = field(compare=False)
    created_at: float = field(compare=False)


@dataclass
class SourceConfig:
    """Per-topic broadcast policy."""

    name: str
    topic: str
    priority: int
    cooldown_sec: float
    prefix: str


class BroadcastManager(Node):
    """Subscribe to text topics, deduplicate them, and speak sequentially."""

    def __init__(self):
        super().__init__('broadcast_manager')

        self._declare_parameters()
        self._queue: List[BroadcastRequest] = []
        self._queue_lock = threading.Condition()
        self._sequence = 0
        self._running = True
        self._last_text_time: Dict[str, float] = {}
        self._last_source_time: Dict[str, float] = {}
        self._subscriptions = []

        self._queue_max_size = max(1, self._get_int('queue_max_size'))
        self._duplicate_window_sec = max(0.0, self._get_float('duplicate_window_sec'))
        self._speak_timeout_sec = max(1.0, self._get_float('speak_timeout_sec'))
        self._speech_enabled = self._get_bool('speech_enabled')

        self._speech = self._init_speech_client()
        self._init_subscriptions()

        self._worker = threading.Thread(
            target=self._worker_loop, name='broadcast_worker', daemon=True)
        self._worker.start()

        self.get_logger().info(
            f'broadcast manager ready, sources={len(self._subscriptions)}, '
            f'speech_enabled={self._speech_enabled and self._speech is not None}')

    def destroy_node(self):
        self._running = False
        with self._queue_lock:
            self._queue_lock.notify_all()
        if hasattr(self, '_worker') and self._worker.is_alive():
            self._worker.join(timeout=1.0)
        super().destroy_node()

    def _declare_parameters(self) -> None:
        self.declare_parameter('source_topics', [
            '/announcement',
            '/image_description',
        ])
        self.declare_parameter('source_names', [
            'announcement',
            'image_description',
        ])
        self.declare_parameter('source_priorities', [80, 60])
        self.declare_parameter('source_cooldowns_sec', [1.0, 8.0])
        self.declare_parameter('source_prefixes', ['', ''])
        self.declare_parameter('queue_max_size', 8)
        self.declare_parameter('duplicate_window_sec', 6.0)
        self.declare_parameter('qos_depth', 10)

        self.declare_parameter('speech_enabled', True)
        self.declare_parameter('log_when_speech_disabled', True)
        self.declare_parameter('i2c_bus', 5)
        self.declare_parameter('i2c_addr', 0x30)
        self.declare_parameter('reader', 'XiaoPing')
        self.declare_parameter('volume', 10)
        self.declare_parameter('speed', 5)
        self.declare_parameter('intonation', 5)
        self.declare_parameter('style', 'Continue')
        self.declare_parameter('language', 'Chinese')
        self.declare_parameter('encoding', 'GB2312')
        self.declare_parameter('configure_on_startup', False)
        self.declare_parameter('wait_after_speak', False)
        self.declare_parameter('speak_timeout_sec', 20.0)

    def _init_speech_client(self):
        if not self._speech_enabled:
            self.get_logger().warn('speech output disabled by parameter')
            return None

        client = None
        try:
            client = SpeechClient(
                bus_id=self._get_int('i2c_bus'),
                i2c_addr=self._get_int('i2c_addr'),
                wait_after_speak=self._get_bool('wait_after_speak'),
                logger=self.get_logger(),
            )
            if self._get_bool('configure_on_startup'):
                client.configure(
                    reader=self.get_parameter('reader').value,
                    volume=self._get_int('volume'),
                    speed=self._get_int('speed'),
                    intonation=self._get_int('intonation'),
                    style=self.get_parameter('style').value,
                    language=self.get_parameter('language').value,
                )
            return client
        except SpeechClientError as err:
            if client is not None:
                self.get_logger().warn(
                    f'speech configure failed, will still try direct speech: {err}')
                return client
            self.get_logger().error(
                f'speech client unavailable, falling back to log-only: {err}')
            return None
        except Exception as err:
            self.get_logger().error(
                f'speech client unavailable, falling back to log-only: {err}')
            return None

    def _init_subscriptions(self) -> None:
        sources = self._build_source_configs()
        qos_depth = max(1, self._get_int('qos_depth'))
        for source in sources:
            sub = self.create_subscription(
                String,
                source.topic,
                lambda msg, src=source: self._on_text(src, msg),
                qos_depth,
            )
            self._subscriptions.append(sub)
            self.get_logger().info(
                f'subscribed {source.topic} as {source.name}, '
                f'priority={source.priority}, cooldown={source.cooldown_sec:.1f}s')

    def _build_source_configs(self) -> List[SourceConfig]:
        topics = list(self.get_parameter('source_topics').value)
        names = self._expand_list(
            list(self.get_parameter('source_names').value), len(topics), 'source')
        priorities = self._expand_list(
            list(self.get_parameter('source_priorities').value), len(topics), 50)
        cooldowns = self._expand_list(
            list(self.get_parameter('source_cooldowns_sec').value), len(topics), 1.0)
        prefixes = self._expand_list(
            list(self.get_parameter('source_prefixes').value), len(topics), '')

        sources = []
        for index, topic in enumerate(topics):
            topic = str(topic).strip()
            if not topic:
                continue
            sources.append(SourceConfig(
                name=str(names[index]),
                topic=topic,
                priority=max(0, min(100, int(priorities[index]))),
                cooldown_sec=max(0.0, float(cooldowns[index])),
                prefix=str(prefixes[index]),
            ))
        return sources

    def _on_text(self, source: SourceConfig, msg: String) -> None:
        text = self._normalize_text(msg.data)
        if not text:
            return

        if source.prefix:
            text = f'{source.prefix}{text}'

        now = time.monotonic()
        if not self._passes_source_cooldown(source, now):
            return
        if not self._passes_duplicate_filter(text, now):
            return

        self._enqueue(source, text, now)

    def _passes_source_cooldown(self, source: SourceConfig, now: float) -> bool:
        last_time = self._last_source_time.get(source.name)
        if last_time is not None and now - last_time < source.cooldown_sec:
            self.get_logger().debug(
                f'drop source cooldown: {source.name}, text within '
                f'{source.cooldown_sec:.1f}s')
            return False
        self._last_source_time[source.name] = now
        return True

    def _passes_duplicate_filter(self, text: str, now: float) -> bool:
        last_time = self._last_text_time.get(text)
        if last_time is not None and now - last_time < self._duplicate_window_sec:
            self.get_logger().info(f'drop duplicate broadcast: {text}')
            return False
        self._last_text_time[text] = now
        self._prune_duplicate_cache(now)
        return True

    def _enqueue(self, source: SourceConfig, text: str, now: float) -> None:
        with self._queue_lock:
            self._sequence += 1
            request = BroadcastRequest(
                priority_key=-source.priority,
                sequence=self._sequence,
                source=source.name,
                text=text,
                created_at=now,
            )

            if len(self._queue) >= self._queue_max_size:
                worst_index = max(
                    range(len(self._queue)),
                    key=lambda idx: (self._queue[idx].priority_key,
                                     -self._queue[idx].sequence),
                )
                worst = self._queue[worst_index]
                if request.priority_key >= worst.priority_key:
                    self.get_logger().warn(
                        f'drop broadcast because queue is full: {text}')
                    return
                self.get_logger().warn(
                    f'replace lower-priority broadcast: {worst.text}')
                self._queue[worst_index] = request
                heapq.heapify(self._queue)
            else:
                heapq.heappush(self._queue, request)

            self.get_logger().info(
                f'queued broadcast from {source.name}: {text}')
            self._queue_lock.notify()

    def _worker_loop(self) -> None:
        while self._running and rclpy.ok():
            request = self._take_next_request()
            if request is None:
                continue
            self._speak_request(request)

    def _take_next_request(self):
        with self._queue_lock:
            while self._running and not self._queue:
                self._queue_lock.wait(timeout=0.2)
            if not self._running:
                return None
            return heapq.heappop(self._queue)

    def _speak_request(self, request: BroadcastRequest) -> None:
        if self._speech is None:
            if self._get_bool('log_when_speech_disabled'):
                self.get_logger().info(
                    f'[broadcast log-only][{request.source}] {request.text}')
            return

        try:
            ok = self._speech.speak_and_wait(
                request.text,
                encoding=self.get_parameter('encoding').value,
                timeout_sec=self._speak_timeout_sec,
            )
            if not ok:
                self.get_logger().warn(
                    f'speech timeout for text: {request.text}')
        except SpeechClientError as err:
            self.get_logger().error(f'speech failed: {err}')

    def _prune_duplicate_cache(self, now: float) -> None:
        expire_before = now - max(self._duplicate_window_sec * 2.0, 30.0)
        old_keys = [
            text for text, ts in self._last_text_time.items()
            if ts < expire_before
        ]
        for text in old_keys:
            del self._last_text_time[text]

    def _get_bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def _get_int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _get_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    @staticmethod
    def _normalize_text(text: str) -> str:
        return ' '.join(str(text).strip().split())

    @staticmethod
    def _expand_list(values, size: int, default):
        if not values:
            values = [default]
        while len(values) < size:
            values.append(values[-1])
        return values[:size]


def main(args=None):
    rclpy.init(args=args)
    node = BroadcastManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
