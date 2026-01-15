#pragma once

#include "ErrorCodes.h"
#include "Structs.h"
#include "Utils.h"
#include <filesystem>
#include <fstream>
#include <queue>
#include <ranges>
#include <limits>

struct InputEntry
{
	std::int64_t src;
	std::int64_t dest;
};

inline std::istream& operator>>(std::istream& in, InputEntry& entry)
{
	in >> entry.src >> entry.dest;

	return in;
}

using ListEntry = std::uint32_t;

struct PQEntry
{
	std::uint32_t id;
	float value;

	friend bool operator>(const PQEntry& l, const PQEntry& r) noexcept
	{
		return l.value > r.value;
	}
};

inline float GetDistance(std::uint32_t uIdx, std::uint32_t vIdx, const std::vector<SavedNode>& nodes)
{
	return EuclideanDistance(nodes[uIdx], nodes[vIdx]);
}

inline void ReconstructBidirectionalPath(
	std::uint32_t meetNode,
	const std::unordered_map<std::uint32_t, std::uint32_t>& parentFwd,
	const std::unordered_map<std::uint32_t, std::uint32_t>& parentBwd,
	const std::vector<SavedNode>& nodes,
	std::vector<std::uint64_t>& route)
{
	std::vector<std::uint64_t> pathStart;
	std::uint32_t curr = meetNode;

	while (true)
	{
		pathStart.push_back(nodes[curr].original_id);
		auto it = parentFwd.find(curr);
		if (it == parentFwd.end()) break;
		curr = it->second;
	}
	std::ranges::reverse(pathStart);

	auto it = parentBwd.find(meetNode);
	if (it != parentBwd.end())
	{
		curr = it->second;
		while (true)
		{
			pathStart.push_back(nodes[curr].original_id);
			auto parIt = parentBwd.find(curr);
			if (parIt == parentBwd.end()) break;
			curr = parIt->second;
		}
	}

	route = std::move(pathStart);
}

inline float BidirectionalAStar(const InputEntry& entry,
	const std::vector<SavedNode>& nodes,
	const std::vector<std::uint32_t>& offsets,
	const std::vector<ListEntry>& allEdges,
	std::vector<std::uint64_t>& route)
{
	const std::uint32_t startIdx = ConvertToArrayIndex(entry.src, nodes);
	const std::uint32_t goalIdx = ConvertToArrayIndex(entry.dest, nodes);

	if (startIdx == static_cast<std::uint32_t>(-1) || goalIdx == static_cast<std::uint32_t>(-1))
	{
		return -1.0f;
	}

	if (startIdx == goalIdx)
	{
		route.push_back(entry.dest);
		return 0.0f;
	}

	const auto& source = nodes[startIdx];
	const auto& destination = nodes[goalIdx];

	const size_t n = nodes.size();
	const float INF = std::numeric_limits<float>::infinity();

	std::unordered_map<std::uint32_t, float> gFwd;
	std::unordered_map<std::uint32_t, float> gBwd;
	std::unordered_map<std::uint32_t, std::uint32_t> parentFwd;
	std::unordered_map<std::uint32_t, std::uint32_t> parentBwd;

	gFwd.reserve(1000); gBwd.reserve(1000);

	std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pqFwd;
	std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pqBwd;

	gFwd[startIdx] = 0.0f;
	gBwd[goalIdx] = 0.0f;

	pqFwd.push({ startIdx, EuclideanDistance(nodes[startIdx], nodes[goalIdx]) });
	pqBwd.push({ goalIdx, EuclideanDistance(nodes[goalIdx], nodes[startIdx]) });

	float mu = INF;
	std::uint32_t meetNode = static_cast<std::uint32_t>(-1);

	while (!pqFwd.empty() && !pqBwd.empty())
	{
		if (pqFwd.top().value + pqBwd.top().value >= mu)
		{
			break;
		}

		if (pqFwd.size() < pqBwd.size())
		{
			auto node = pqFwd.top(); pqFwd.pop();
			std::uint32_t u = node.id;

			if (node.value > gFwd[u] + EuclideanDistance(nodes[u], nodes[goalIdx]) + 1e-5f)
				continue;

			for (std::uint32_t i = offsets[u]; i < offsets[u + 1]; ++i)
			{
				std::uint32_t v = allEdges[i];
				float w = GetDistance(u, v, nodes);
				float newG = gFwd[u] + w;

				auto it = gFwd.find(v);
				float oldG = (it == gFwd.end()) ? INF : it->second;

				if (newG < oldG)
				{
					gFwd[v] = newG;
					parentFwd[v] = u;
					pqFwd.push({ v, newG + EuclideanDistance(nodes[v], nodes[goalIdx]) });

					auto itBwd = gBwd.find(v);
					if (itBwd != gBwd.end())
					{
						float dist = newG + itBwd->second;
						if (dist < mu)
						{
							mu = dist;
							meetNode = v;
						}
					}
				}
			}
		}
		else
		{
			auto node = pqBwd.top(); pqBwd.pop();
			std::uint32_t u = node.id;

			if (node.value > gBwd[u] + EuclideanDistance(nodes[u], nodes[startIdx]) + 1e-5f)
				continue;

			for (std::uint32_t i = offsets[u]; i < offsets[u + 1]; ++i)
			{
				std::uint32_t v = allEdges[i];
				float w = GetDistance(u, v, nodes);
				float newG = gBwd[u] + w;

				auto it = gBwd.find(v);
				float oldG = (it == gBwd.end()) ? INF : it->second;

				if (newG < oldG)
				{
					gBwd[v] = newG;
					parentBwd[v] = u;
					pqBwd.push({ v, newG + EuclideanDistance(nodes[v], nodes[startIdx]) });

					auto itFwd = gFwd.find(v);
					if (itFwd != gFwd.end())
					{
						float dist = newG + itFwd->second;
						if (dist < mu)
						{
							mu = dist;
							meetNode = v;
						}
					}
				}
			}
		}
	}

	if (meetNode != static_cast<std::uint32_t>(-1))
	{
		route.clear();
		ReconstructBidirectionalPath(meetNode, parentFwd, parentBwd, nodes, route);
		return mu;
	}

	return -1.0f;
}

inline int DoSearch(std::filesystem::path graphPath,
	std::filesystem::path inputPath,
	const std::filesystem::path& outputPath,
	bool full_output)
{
	std::ifstream graphFile{ graphPath, std::ios::binary };

	if (!graphFile)
		return ERR_NOT_FOUND;

	size_t nodeCount = 0;
	if (!graphFile.read(reinterpret_cast<char*>(&nodeCount), sizeof(size_t)))
	{
		return ERR_CORRUPTED_DATA;
	}

	std::vector<SavedNode> nodes(nodeCount);
	if(!graphFile.read(reinterpret_cast<char*>(nodes.data()), sizeof(SavedNode) * nodeCount))
	{
		return ERR_CORRUPTED_DATA;
	}

	std::vector<std::uint32_t> offsets(nodeCount + 1, 0);
	auto edgesStartPos = graphFile.tellg();

	SavedEdge dummyEdge;
	while (graphFile.read(reinterpret_cast<char*>(&dummyEdge), sizeof(SavedEdge)))
	{
		if (dummyEdge.source < nodeCount && dummyEdge.destination < nodeCount)
		{
			offsets[dummyEdge.source + 1]++;
			offsets[dummyEdge.destination + 1]++;
		}
	}

	for (size_t i = 0; i < nodeCount; ++i)
	{
		offsets[i + 1] += offsets[i];
	}

	std::vector<ListEntry> allEdges(offsets.back());
	graphFile.clear();
	graphFile.seekg(edgesStartPos);

	while (graphFile.read(reinterpret_cast<char*>(&dummyEdge), sizeof(SavedEdge)))
	{
		if (dummyEdge.source < nodeCount && dummyEdge.destination < nodeCount)
		{
			allEdges[offsets[dummyEdge.source]++] = dummyEdge.destination;
			allEdges[offsets[dummyEdge.destination]++] = dummyEdge.source;
		}
	}

	graphFile.close();

	for (size_t i = nodeCount; i > 0; --i)
	{
		offsets[i] = offsets[i - 1];
	}
	offsets[0] = 0;

	std::ifstream inputFile{ inputPath };
	if (!inputFile)
		return ERR_NOT_FOUND;

	std::ofstream outputFile{ outputPath };
	if (!outputFile)
		return ERR_NOT_FOUND;

	outputFile.precision(5);
	outputFile << std::fixed;

	{
		std::uint64_t nQueries = 0;
		inputFile >> nQueries;
		for(std::uint64_t i = 0; i < nQueries; ++i)
		{
			InputEntry entry;
			inputFile >> entry;

			std::vector<std::uint64_t> route;
			float routeLength = -1.0f;
			
			if (entry.src >= 0 && entry.dest >= 0)
			{
				routeLength = BidirectionalAStar(entry, nodes, offsets, allEdges, route);
			} 

			if (routeLength < 0.0f)
			{
				outputFile << "-1 0\n";
				continue;
			}

			outputFile << routeLength;
			if (full_output)
			{
				outputFile << " ";
				std::ranges::copy(route, std::ostream_iterator<std::uint64_t>(outputFile, " "));
			}
			outputFile << "\n";
		}
	}

	inputFile.close();
	outputFile.close();

	return S_OK;
}