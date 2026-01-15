#pragma once

#pragma pack(push, 1)
struct SavedNode
{
	std::uint32_t original_id;
	float lat;
	float lon;

	bool operator==(const SavedNode&) const = default;
};

struct SavedEdge
{
	std::uint32_t source;
	std::uint32_t destination;
	float dist;
};
#pragma pack(pop)

inline std::istream& operator>>(std::istream& is, SavedNode& node)
{
	is >> node.original_id >> node.lat >> node.lon;

	return is;
}

inline std::istream& operator>>(std::istream& is, SavedEdge& edge)
{
	is >> edge.source >> edge.destination >> edge.dist;

	return is;
}