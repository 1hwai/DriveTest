#include "StageEditor.h"

#include "../World/StageSerializer.h"

void StageEditor::SetStage(StageDefinition* stage) {
    m_stage = stage;
    m_selectedRoadPoint = 0;
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

void StageEditor::SetName(const std::string& name) {
    if (!m_stage || name.empty())
        return;

    m_stage->name = name;
}

void StageEditor::SetRoadWidth(float width) {
    if (!m_stage || width <= 0.0f)
        return;

    m_stage->roadWidth = width;
}

size_t StageEditor::GetSelectedRoadPointIndex() const {
    return m_selectedRoadPoint;
}

Vec3* StageEditor::GetSelectedRoadPoint() {
    if (!m_stage || m_stage->roadControlPoints.empty() ||
        m_selectedRoadPoint >= m_stage->roadControlPoints.size())
        return nullptr;

    return &m_stage->roadControlPoints[m_selectedRoadPoint];
}

const Vec3* StageEditor::GetSelectedRoadPoint() const {
    if (!m_stage || m_stage->roadControlPoints.empty() ||
        m_selectedRoadPoint >= m_stage->roadControlPoints.size())
        return nullptr;

    return &m_stage->roadControlPoints[m_selectedRoadPoint];
}

bool StageEditor::SelectNextRoadPoint() {
    if (!m_stage || m_stage->roadControlPoints.empty())
        return false;

    m_selectedRoadPoint =
        (m_selectedRoadPoint + 1) %
        m_stage->roadControlPoints.size();

    return true;
}

bool StageEditor::SelectPreviousRoadPoint() {
    if (!m_stage || m_stage->roadControlPoints.empty())
        return false;

    if (m_selectedRoadPoint == 0)
        m_selectedRoadPoint = m_stage->roadControlPoints.size() - 1;
    else
        --m_selectedRoadPoint;

    return true;
}

bool StageEditor::MoveSelectedRoadPoint(const Vec3& delta) {
    Vec3* point = GetSelectedRoadPoint();
    if (!point)
        return false;

    *point += delta;
    return true;
}

bool StageEditor::AddRoadPoint(const Vec3& point) {
    if (!m_stage)
        return false;

    m_stage->roadControlPoints.push_back(point);
    m_selectedRoadPoint = m_stage->roadControlPoints.size() - 1;
    return true;
}

bool StageEditor::DeleteSelectedRoadPoint() {
    if (!m_stage || m_stage->roadControlPoints.size() <= 2 ||
        m_selectedRoadPoint >= m_stage->roadControlPoints.size())
        return false;

    m_stage->roadControlPoints.erase(
        m_stage->roadControlPoints.begin() + m_selectedRoadPoint
    );

    if (m_selectedRoadPoint >= m_stage->roadControlPoints.size())
        m_selectedRoadPoint = m_stage->roadControlPoints.size() - 1;

    return true;
}