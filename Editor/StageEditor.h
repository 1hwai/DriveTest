#pragma once

#include <cstddef>
#include <string>

#include "../Core/Math/Vec3.h"

struct StageDefinition;

class StageEditor {
public:
    void SetStage(StageDefinition* stage);

    StageDefinition* GetStage();
    const StageDefinition* GetStage() const;

    bool HasStage() const;

    void SetName(const std::string& name);
    void SetRoadWidth(float width);

    size_t GetSelectedRoadPointIndex() const;
    Vec3* GetSelectedRoadPoint();
    const Vec3* GetSelectedRoadPoint() const;

    bool SelectNextRoadPoint();
    bool SelectPreviousRoadPoint();
    bool MoveSelectedRoadPoint(const Vec3& delta);
    bool AddRoadPoint(const Vec3& point);
    bool DeleteSelectedRoadPoint();

private:
    StageDefinition* m_stage = nullptr;
    size_t m_selectedRoadPoint = 0;
};