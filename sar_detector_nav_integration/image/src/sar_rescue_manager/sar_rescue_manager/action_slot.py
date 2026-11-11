"""DETECTOR_NAV_INTEGRATION: one owned action, including pending acceptance/cancellation."""
import time


class ActionSlot:
    def __init__(self, client, terminal, fault):
        self.client, self.terminal, self.fault = client, terminal, fault
        self.busy = False
        self.handle = None
        self.cancel_requested = False
        self.cancel_sent = False
        self.started = 0.
        self.cancel_started = None
        self.label = None

    def send(self, goal, label):
        if self.busy:
            raise RuntimeError('Cannot replace a goal before terminal result')
        self.busy, self.handle = True, None
        self.cancel_requested = self.cancel_sent = False
        self.started = time.monotonic()
        self.cancel_started = None
        self.label = label
        try:
            self.client.send_goal_async(goal).add_done_callback(self._accepted)
        except Exception as exc:
            self.fault(f'Action submission outcome unknown: {exc}')

    def _accepted(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self._finish(6, None)  # rejected; no executing goal exists
                return
            self.handle = handle
            handle.get_result_async().add_done_callback(self._result)
            if self.cancel_requested:
                self.cancel()
        except Exception as exc:
            self.fault(f'Action acceptance/result registration failed: {exc}')

    def _result(self, future):
        try:
            response = future.result()
            self._finish(response.status, response.result)
        except Exception as exc:
            self.fault(f'Action terminal state unknown: {exc}')

    def _finish(self, status, result):
        label = self.label
        self.busy, self.handle = False, None
        self.terminal(label, status, result)

    def cancel(self):
        if not self.busy:
            return
        self.cancel_requested = True
        if self.cancel_started is None:
            self.cancel_started = time.monotonic()
        if self.handle is not None and not self.cancel_sent:
            self.cancel_sent = True
            try:
                self.handle.cancel_goal_async().add_done_callback(self._cancel_ack)
            except Exception as exc:
                self.fault(f'Cancel submission failed: {exc}')

    def _cancel_ack(self, future):
        try:
            # Acknowledgement does not release ownership. Only terminal result does.
            response = future.result()
            if not response.goals_canceling and self.busy:
                self.fault('Cancel rejected while action still has no terminal result')
        except Exception as exc:
            self.fault(f'Cancel response failed: {exc}')

    def check(self, timeout, cancel_timeout):
        if not self.busy:
            return
        now = time.monotonic()
        if self.cancel_started is not None and now-self.cancel_started > cancel_timeout:
            self.fault('Cancellation/acceptance timed out; ownership remains held')
        elif now-self.started > timeout:
            self.cancel()
