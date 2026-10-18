"""PC-side authenticated TCP transport. No SSH, no Unitree SDK, no automatic reconnect."""
import collections
import json
import re
import select
import socket
import threading
import time
from timeouts import NETWORK_TIMEOUT_S, UI_LEASE_S, ACTION_QUEUE_S


class RobotLink:
    def __init__(self, config):
        self.config = config
        self.lock = threading.RLock()
        self.socket = None
        self.thread = None
        self.stopping = threading.Event()
        self.connected = False
        self.connecting = False
        self.robot = {}
        self.received_at = 0.
        self.ui_until = 0.
        self.held_key = ''
        self.events = collections.deque(maxlen=12)
        self.queue = collections.deque(maxlen=8)
        self.force_stop = False
        self.stop_only = False
        self.sequence = 0
        self.rtt_ms = None
        self.sent = {}
        self.recovery_required = False
        self.degraded = False
        self.disconnect_reason = ''
        self.connection_stage = 'idle'
        self.last_fault = None
        self.last_disconnect = None
        self.last_network_gap = None
        self.last_ui_heartbeat = 0.
        self.max_ui_gap_ms = 0.
        self.ui_expirations = 0
        self.simulation = bool(config.get('simulation', False))
        if not re.fullmatch('[0-9a-f]{64}', config.get('token', '')):
            raise ValueError('配对 token 必须是 64 位小写十六进制；先运行 tools/create_pairing.py')
        if self.simulation and config['host'] != '127.0.0.1':
            raise ValueError('模拟模式只允许连接 127.0.0.1')

    def log(self, message):
        self.events.append({'time': time.strftime('%H:%M:%S'), 'message': message})

    def connect(self):
        with self.lock:
            if self.connected or self.connecting:
                raise ValueError('已连接或正在连接')
            if self.thread and self.thread.is_alive():
                raise ValueError('上一连接正在释放，请稍后重试')
            self.connecting = True
            self.stopping.clear()
            self.ui_until = 0
            self.held_key = ''
            self.queue.clear()
            self.force_stop = True
            self.robot = {}
            self.received_at = 0.
            self.recovery_required = False
            self.degraded = False
            self.disconnect_reason = ''
            self.connection_stage = 'connecting'
            self.stop_only = False
            self.rtt_ms = None
            self.last_ui_heartbeat = self.max_ui_gap_ms = 0.
            self.ui_expirations = 0
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def submit(self, action, key='', speed=.5, standing=False, lateral=.4, turn=.6):
        with self.lock:
            if action == 'connect':
                self.connect()
                return
            if action == 'disconnect':
                self.held_key = ''
                self.ui_until = 0
                self.last_ui_heartbeat = 0.
                self.force_stop = True
                self.stopping.set()
                self.disconnect_reason = '用户已断开；需要恢复后手动连接'
                return
            if action == 'disarm':
                self.held_key = ''
                self.ui_until = 0
                self.last_ui_heartbeat = 0.
                self.force_stop = True
                self.queue.clear()
                return
            if action == 'stop':
                self.held_key = ''
                self.stop_only = True
                self.queue.clear()
                return
            if not self.connected or time.monotonic()-self.received_at > NETWORK_TIMEOUT_S:
                raise ValueError('连接未就绪或状态已过期')
            if action == 'heartbeat':
                if key not in ('', 'w', 's', 'j', 'l', 'z', 'c'):
                    raise ValueError('无效方向键')
                if self.recovery_required:
                    key = ''
                if self.held_key and not key:
                    self.stop_only = True
                now = time.monotonic()
                if self.last_ui_heartbeat and self.ui_until:
                    self.max_ui_gap_ms = max(self.max_ui_gap_ms, (now-self.last_ui_heartbeat)*1000)
                self.last_ui_heartbeat = now
                self.ui_until = now+UI_LEASE_S
                self.held_key = key
                return
            mapping = {'take': 'TAKE', 'arm': 'ARM', 'reset': 'RESET', 'avoid': 'AVOID', 'release': 'RELEASE'}
            if action not in mapping:
                raise ValueError('未知操作')
            if len(self.queue) >= 8:
                raise ValueError('操作队列已满')
            if action == 'take':
                if standing is not True or isinstance(speed, bool) or not isinstance(speed, (int, float)) or not .05 <= speed <= .5:
                    raise ValueError('接管需确认已站立、已拔充电器；前后速度须为 0.05–0.5 m/s')
                if (type(lateral) not in (int, float) or not .1 <= lateral <= .4 or
                        type(turn) not in (int, float) or not .1 <= turn <= .6):
                    raise ValueError('横移须为 0.1–0.4 m/s；转向须为 0.1–0.6 rad/s')
                if not self.robot.get('configurable_speeds'):
                    raise ValueError('接收端仍是旧版本；请先更新机器狗接收端再接管')
                if self.robot.get('control_timeout_ms') != 3000 or self.robot.get('command_echo_ms') != 3000:
                    raise ValueError('接收端仍使用旧网络阈值；请更新为 3 秒超时版本后再接管')
                if self.robot.get('hold_to_move') is not True or self.robot.get('continuous_limit_s', 'unknown') is not None:
                    raise ValueError('接收端尚未移除连续运动时限；请更新为 v0.3.1 后再接管')
                if self.robot.get('steady_motion_renewal') is not True:
                    raise ValueError('接收端仍可能因轻微横摆中断续行，请更新为 v0.3.2 后再接管')
                if self.robot.get('startup_ack_ms') != 1500:
                    raise ValueError('接收端启动确认窗口仍是旧值，请更新为 v0.3.3 后再接管')
                arg = 'STANDING,{},{},{}'.format(round(speed*1000), round(lateral*1000), round(turn*1000))
            else:
                arg = '-'
            self.ui_until = time.monotonic()+UI_LEASE_S
            if action in ('take', 'arm'):
                self.recovery_required = False  # Only a fresh, explicit manual request can clear this latch.
                self.last_ui_heartbeat = 0.
            if action in ('take', 'reset', 'avoid', 'release'):
                self.held_key = ''
            self.queue.append((mapping[action], arg, time.monotonic()+ACTION_QUEUE_S))

    def snapshot(self):
        with self.lock:
            age = (time.monotonic()-self.received_at)*1000 if self.received_at else None
            fresh = self.connected and age is not None and age <= NETWORK_TIMEOUT_S*1000
            stage = 'online' if fresh else 'stale' if self.connected else self.connection_stage
            return {'connected': self.connected, 'connecting': self.connecting,
                    'connection_stage': stage, 'robot_fresh': fresh,
                    'address': str(self.config['host'])+':'+str(self.config['port']),
                    'simulation': self.simulation, 'telemetry_age_ms': age, 'rtt_ms': self.rtt_ms,
                    'network_timeout_ms': int(NETWORK_TIMEOUT_S*1000),
                    'degraded': self.connected and (age is None or age > NETWORK_TIMEOUT_S*1000),
                    'recovery_required': self.recovery_required, 'disconnect_reason': self.disconnect_reason,
                    'ui_lease_active': time.monotonic() < self.ui_until,
                    'input_diagnostics': {'heartbeat_age_ms': (time.monotonic()-self.last_ui_heartbeat)*1000 if self.last_ui_heartbeat else None,
                                          'max_heartbeat_gap_ms': self.max_ui_gap_ms, 'ui_expirations': self.ui_expirations},
                    'robot': dict(self.robot) if fresh else {}, 'events': list(self.events),
                    'history': {'last_fault': dict(self.last_fault) if self.last_fault else None,
                                'last_disconnect': dict(self.last_disconnect) if self.last_disconnect else None,
                                'last_network_gap': dict(self.last_network_gap) if self.last_network_gap else None}}

    def _accept_status(self, state):
        """Caller holds lock. Store observations separately from current control authority."""
        previous_reason = self.robot.get('fault_reason') if self.robot.get('fault') else None
        previous_session = self.robot.get('session')
        previous_stops = self.robot.get('motion_stop_count', 0) if previous_session == state.get('session') else 0
        self.robot = state
        self.received_at = time.monotonic()
        self.connected = state.get('authenticated') is True
        self.connecting = not self.connected
        self.connection_stage = 'online' if self.connected else 'authenticating'
        if self.connected and state.get('motion_stop_count', 0) > previous_stops:
            self.log('运动目标归零：{}；指令间隔 {} ms；TTL 中断累计 {}'.format(
                state.get('last_stop_reason', 'UNKNOWN'), state.get('move_gap_ms', '?'), state.get('ttl_expirations', '?')))
        reason = state.get('fault_reason') if state.get('fault') else None
        if self.connected and reason and (not self.last_fault or previous_reason != reason or previous_session != state.get('session')):
            self.last_fault = {'time': time.strftime('%H:%M:%S'), 'reason': reason}
        if self.connected and not state.get('control_active') and not state.get('stopping'):
            self.recovery_required = False  # No SDK session can resume; a new TAKE and ARM remain mandatory.

    def _expire_state(self, now):
        """Stop authority on stale status; an idle monitoring gap must not latch an arm warning."""
        stale = self.connected and now-self.received_at > NETWORK_TIMEOUT_S
        if stale:
            had_authority = bool(self.robot.get('control_active') or self.robot.get('armed') or self.held_key or
                                 any(op in ('TAKE', 'ARM') for op, _, _ in self.queue))
            self.held_key = ''
            self.ui_until = 0
            self.queue.clear()
            self.recovery_required = self.recovery_required or had_authority
            if not self.degraded:
                self.force_stop = True
                self.last_network_gap = {'time': time.strftime('%H:%M:%S'), 'age_ms': round((now-self.received_at)*1000)}
                self.log('连续 3 秒无状态反馈：停止续发；历史记录不代表当前故障')
        self.degraded = stale

    def _clear_connection(self):
        """Clear all live fields; preserve only explicitly labelled diagnostic history."""
        self.socket = None
        self.connected = self.connecting = False
        self.connection_stage = 'offline'
        self.robot = {}
        self.received_at = 0.
        self.rtt_ms = None
        self.held_key = ''
        self.ui_until = 0
        self.last_ui_heartbeat = 0.
        self.queue.clear()
        self.sent.clear()
        self.force_stop = True
        self.stop_only = False
        self.degraded = self.recovery_required = False
        if self.disconnect_reason:
            self.last_disconnect = {'time': time.strftime('%H:%M:%S'), 'reason': self.disconnect_reason}

    def close(self):
        self.submit('disconnect')
        if self.thread:
            self.thread.join(3)

    def _frame(self, op, arg, ui_active):
        self.sequence += 1
        self.sent[self.sequence] = time.monotonic()
        self.sent = {k: v for k, v in self.sent.items() if time.monotonic()-v < NETWORK_TIMEOUT_S*2}
        return ('G1R1|{token}|{session}|{seq}|{tick}|{op}|{arg}|{ui}\n'.format(
            token=self.config['token'], session=self.robot['session'], seq=self.sequence,
            tick=self.robot['tick'], op=op, arg=arg, ui=int(ui_active))).encode('ascii')

    def _run(self):
        connection = None
        try:
            connection = socket.create_connection((self.config['host'], int(self.config['port'])), timeout=NETWORK_TIMEOUT_S)
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            connection.settimeout(.25)
            with self.lock:
                self.socket = connection
                self.connection_stage = 'authenticating'
                self.sequence = 0
                self.sent.clear()
            buffer = b''
            authed_sent = False
            last_send = 0.
            started = time.monotonic()
            while not self.stopping.is_set():
                if select.select([connection], [], [], .01)[0]:
                    data = connection.recv(8192)
                    if not data:
                        raise ConnectionError('机器狗接收端已断开；若刚连接就断开，请检查是否有另一电脑服务占用')
                    buffer += data
                    if len(buffer) > 16384:
                        raise ConnectionError('状态帧超过长度限制')
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        state = json.loads(line)
                        if state.get('protocol') != 1 or not re.fullmatch('[0-9a-f]{16}', state.get('session', '')):
                            raise ConnectionError('状态协议错误')
                        if bool(state.get('simulated')) != self.simulation:
                            raise ConnectionError('模拟/实机模式不匹配，已拒绝连接')
                        with self.lock:
                            if self.robot and state['session'] != self.robot['session']:
                                raise ConnectionError('接收端会话已改变')
                            self._accept_status(state)
                            if state.get('ack_seq') in self.sent:
                                self.rtt_ms = (time.monotonic()-self.sent.pop(state['ack_seq']))*1000
                            if not authed_sent:
                                connection.sendall(self._frame('AUTH', '-', False))
                                authed_sent = True
                            if self.connected and not any(e['message'] == '网络已连接；尚未自动接管或解锁' for e in self.events):
                                self.log('网络已连接；尚未自动接管或解锁')
                now = time.monotonic()
                with self.lock:
                    if not self.connected and now-started > NETWORK_TIMEOUT_S:
                        raise ConnectionError('认证超时或配对密钥错误')
                    self._expire_state(now)
                    if self.connected and now-self.received_at > NETWORK_TIMEOUT_S:
                        # Stop is independent of receive freshness; never resend a queued MOVE on exit.
                        connection.sendall(self._frame('DISARM', '-', False))
                        raise ConnectionError('机器狗状态链路连续 3 秒无反馈；连接已关闭，需手动重连')
                    if self.connected and now-last_send >= .05:
                        ui_active = now < self.ui_until
                        if self.force_stop or (not ui_active and self.held_key):
                            if not ui_active and self.held_key:
                                self.ui_expirations += 1
                                self.log('浏览器按键心跳已失效：请求解除解锁；不是运动时长上限')
                            self.held_key = ''
                            if not ui_active:
                                self.queue.clear()
                            op, arg = 'DISARM', '-'
                            self.force_stop = False
                            self.stop_only = False
                        elif self.stop_only:
                            op, arg = 'STOP', '-'
                            self.stop_only = False
                        elif self.queue:
                            op, arg, deadline = self.queue.popleft()
                            if now > deadline or not ui_active:
                                op, arg = 'DISARM', '-'
                        elif ui_active and self.held_key:
                            op, arg = 'MOVE', self.held_key
                        else:
                            op, arg = 'PING', '-'
                        connection.sendall(self._frame(op, arg, ui_active))
                        last_send = now
            with self.lock:
                if self.connected and self.robot:
                    try:
                        connection.sendall(self._frame('RELEASE', '-', False))
                    except OSError:
                        pass
        except (OSError, ValueError, KeyError, ConnectionError) as error:
            with self.lock:
                if isinstance(error, (socket.timeout, TimeoutError)):
                    self.disconnect_reason = '无法连接接收端：连接超时。请确认机器狗已开机并连接其热点；关机或充电期间不要接管。'
                elif isinstance(error, ConnectionRefusedError):
                    self.disconnect_reason = '接收端拒绝连接：可能未开机、服务未监听或地址不可达。不是已确认的机器人控制故障。'
                elif isinstance(error, ConnectionResetError):
                    self.disconnect_reason = '连接被远端重置：可能关机、链路中断或服务退出；具体原因需恢复连接后检查。'
                else:
                    self.disconnect_reason = str(error)
                self.log('连接结束：'+str(error))
        finally:
            if connection:
                connection.close()
            with self.lock:
                self._clear_connection()
