#pragma once

#include "Structs.h"
#include <cstdint>
#include <ranges>
#include <vector>

inline std::int64_t ConvertToArrayIndex(std::uint32_t indexFromFile, const std::vector<SavedNode>& nodes)
{
	auto it = std::ranges::lower_bound(nodes, indexFromFile, {}, &SavedNode::original_id);

	if (it == std::end(nodes))
	{
		return -1;
	}

	return std::distance(std::begin(nodes), it);
}

inline float EuclideanDistance(const SavedNode& n1, const SavedNode& n2)
{
	float dx = n2.lat - n1.lat;
	float dy = n2.lon - n1.lon;
	return std::sqrt(dx * dx + dy * dy);
}