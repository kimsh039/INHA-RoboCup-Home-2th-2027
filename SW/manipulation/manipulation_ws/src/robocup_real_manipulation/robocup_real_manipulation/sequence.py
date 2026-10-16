"""One non-retrying motion sequence. Only observations may retry."""
from .core import Fault, STAGES

class Sequence:
    def __init__(self, backend):
        self.b=backend; self.stage='WAIT_FOR_APPROACH'; self.holding='EMPTY'
        self.place_success=False; self.return_success=False; self.stop_confirmed=False
    def run(self):
        outcome='FAILED'; reason=''
        try:
            stages = list(STAGES)
            handle = getattr(self.b, 'goal_handle', None)
            if handle and getattr(handle.request, 'operation', '') == 'table_plate':
                stages.insert(stages.index('MOVE_TO_PLACE'), 'OBSERVE_PLATE')
            for stage in stages:
                self.stage=stage; self.b.status(self,'RUNNING','')
                self.b.guard()
                if stage=='CLOSE_GRIPPER': self.holding='UNKNOWN'
                self.b.perform(stage,self)
                if stage=='VERIFY_GRASP': self.holding='HELD'
                if stage=='RELEASE': self.holding='EMPTY'; self.place_success=True
                if stage=='RETURN_ARM': self.return_success=True
            if hasattr(self.b,'finish_success'): reason=self.b.finish_success(self) or ''
            outcome='SUCCESS'
        except Exception as e:
            reason=str(e); outcome='CANCELED' if reason=='CANCELED' else 'FAILED'
            try: self.stop_confirmed=self.b.stop()
            except Exception: self.stop_confirmed=False
            # Preserve object and base lock. Recovery requires operator intervention.
        self.b.status(self,outcome,reason)
        return outcome,reason
