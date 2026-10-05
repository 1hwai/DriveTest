#include "StageEditor.h"

void StageEditor::SetStage(StageDefinition* stage) {
    m_stage = stage;
}

StageDefinition* StageEditor::GetStage() {
    return m_stage;
}

const StageDefinition* StageEditor::GetStage() const {
    return m_stage;
}

bool StageEditor::HasStage() const {
    return m_stage != nullptr;
}