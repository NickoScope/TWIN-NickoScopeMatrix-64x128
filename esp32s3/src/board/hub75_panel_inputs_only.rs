//! TEMPORARY `--board hub75-panel`: the NickoScope LED panel with its inputs only (IR receiver,
//! EC11 knob, BOOT; `panel_inputs.rs`) and no display, so the inputs can be exercised before the
//! HUB75 model exists. The real board embeds `PanelInputs` the same way and forwards the same
//! eight methods; when it lands, delete this file and its two lines in `mod.rs`.
use super::panel_inputs::PanelInputs;
use esp_soc::board::{BoardEdge, BoardModel, VirtualCycle};

#[derive(Default)]
pub struct Hub75PanelInputsOnly { pub inputs: PanelInputs }

impl BoardModel for Hub75PanelInputsOnly {
    fn name(&self) -> &'static str { "hub75-panel" }
    fn input_levels(&self) -> Vec<(u8, bool)> { self.inputs.input_levels() }
    fn next_deadline(&self) -> Option<VirtualCycle> { self.inputs.next_deadline() }
    fn advance_to(&mut self, cycle: VirtualCycle) { self.inputs.advance_to(cycle) }
    fn take_edges(&mut self) -> Vec<BoardEdge> { self.inputs.take_edges() }
    fn check_input(&self, cmd: &str, args: &str) -> Option<Result<(), String>> { self.inputs.check_input(cmd, args) }
    fn input_at(&mut self, cycle: VirtualCycle, cmd: &str, args: &str) -> Result<(), String> { self.inputs.input_at(cycle, cmd, args) }
    fn report(&self) -> String { self.inputs.report() }
}
