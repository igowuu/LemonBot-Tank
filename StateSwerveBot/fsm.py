from typing import Callable, Optional, List
from dataclasses import dataclass

@dataclass
class State:
    name: str
    conditional: Callable[..., bool]
    on_update: Callable[..., bool]
    on_enable: Optional[Callable[..., bool]] = None
    on_disable: Optional[Callable[..., bool]] = None

class FiniteStateMachine:
    def __init__(self) -> None:
        self.states: List[State] = []
        self.default_state = None
        self.current_state = None

    def set_default_state(
        self,
        name: str,
        on_update: Callable[..., bool],
        on_enable: Optional[Callable[..., bool]] = None,
        on_disable: Optional[Callable[..., bool]] = None,
    ):
        conditional = lambda _: True
        self.default_state = (
            State(
                name, conditional, on_update, on_enable, on_disable
            )
        )

    def add_state(
        self,
        name: str,
        conditional: Callable[..., bool],
        on_update: Callable[..., bool],
        on_enable: Optional[Callable[..., bool]] = None,
        on_disable: Optional[Callable[..., bool]] = None,
    ) -> None:
        self.states.append(
            State(
                name, conditional, on_update, on_enable, on_disable
            )
        )

    def execute(self) -> None:
        if self.current_state is None:
            self.current_state = self.default_state
            if self.current_state.on_enable:
                self.current_state.on_enable()

        for state in self.states:
            if state.conditional():
                if state != self.current_state:
                    if self.current_state and self.current_state.on_disable:
                        self.current_state.on_disable()

                    self.current_state = state
                    if self.current_state.on_enable:
                        self.current_state.on_enable()
                break
        else:
            if self.current_state != self.default_state:
                if self.current_state.on_disable:
                    self.current_state.on_disable()

                self.current_state = self.default_state
                if self.current_state.on_enable:
                    self.current_state.on_enable()

        if self.current_state and self.current_state.on_update:
            self.current_state.on_update()