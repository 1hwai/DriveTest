#pragma once

#include <string>

struct StageDefinition;

class StageEditor {
public:
    void SetStage(StageDefinition* stage);

    StageDefinition* GetStage();
    const StageDefinition* GetStage() const;

    bool HasStage() const;

private:
    StageDefinition* m_stage = nullptr;
};