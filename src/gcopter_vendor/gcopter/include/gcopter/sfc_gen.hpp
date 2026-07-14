/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Path frontend: grid A* (no OMPL) for drone_ws Path C.
    convexCover / shortCut retained from upstream GCOPTER.
*/

#ifndef SFC_GEN_HPP
#define SFC_GEN_HPP

#include "geo_utils.hpp"
#include "firi.hpp"

#include <deque>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <Eigen/Eigen>

namespace sfc_gen
{

    // Grid A* frontend (replaces OMPL InformedRRTstar for Humble builds without libompl).
    template <typename Map>
    inline double planPath(const Eigen::Vector3d &s,
                           const Eigen::Vector3d &g,
                           const Eigen::Vector3d &lb,
                           const Eigen::Vector3d &hb,
                           Map *mapPtr,
                           const double & /*timeout*/,
                           std::vector<Eigen::Vector3d> &p)
    {
        p.clear();
        const double res = std::max(0.25, mapPtr->getScale());
        auto toKey = [&](const Eigen::Vector3i &idx) -> int64_t {
            return (static_cast<int64_t>(idx.x()) << 42) ^
                   (static_cast<int64_t>(idx.y()) << 21) ^
                   static_cast<int64_t>(idx.z());
        };
        auto worldToIdx = [&](const Eigen::Vector3d &w) -> Eigen::Vector3i {
            return Eigen::Vector3i(
                static_cast<int>(std::floor((w.x() - lb.x()) / res)),
                static_cast<int>(std::floor((w.y() - lb.y()) / res)),
                static_cast<int>(std::floor((w.z() - lb.z()) / res)));
        };
        auto idxToWorld = [&](const Eigen::Vector3i &idx) -> Eigen::Vector3d {
            return Eigen::Vector3d(
                lb.x() + (idx.x() + 0.5) * res,
                lb.y() + (idx.y() + 0.5) * res,
                lb.z() + (idx.z() + 0.5) * res);
        };
        const Eigen::Vector3i bmax(
            std::max(1, static_cast<int>(std::floor((hb.x() - lb.x()) / res)) - 1),
            std::max(1, static_cast<int>(std::floor((hb.y() - lb.y()) / res)) - 1),
            std::max(1, static_cast<int>(std::floor((hb.z() - lb.z()) / res)) - 1));

        auto inBound = [&](const Eigen::Vector3i &i) {
            return i.x() >= 0 && i.y() >= 0 && i.z() >= 0 &&
                   i.x() <= bmax.x() && i.y() <= bmax.y() && i.z() <= bmax.z();
        };
        auto free = [&](const Eigen::Vector3i &i) {
            if (!inBound(i)) return false;
            return mapPtr->query(idxToWorld(i)) == 0;
        };

        Eigen::Vector3i start = worldToIdx(s);
        Eigen::Vector3i goal = worldToIdx(g);
        const int z0 = start.z();
        goal.z() = z0;

        auto nudge = [&](Eigen::Vector3i &cell) {
            if (free(cell)) return true;
            for (int r = 1; r <= 6; ++r) {
                for (int dx = -r; dx <= r; ++dx) {
                    for (int dy = -r; dy <= r; ++dy) {
                        Eigen::Vector3i c(cell.x() + dx, cell.y() + dy, z0);
                        if (free(c)) { cell = c; return true; }
                    }
                }
            }
            return false;
        };
        if (!nudge(start) || !nudge(goal)) {
            p.push_back(s);
            p.push_back(g);
            return (g - s).norm();
        }

        struct Node {
            Eigen::Vector3i idx;
            double g, f;
            bool operator>(const Node &o) const { return f > o.f; }
        };
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
        std::unordered_map<int64_t, double> gscore;
        std::unordered_map<int64_t, Eigen::Vector3i> parent;
        auto heur = [&](const Eigen::Vector3i &a) {
            return (idxToWorld(a) - idxToWorld(goal)).cwiseAbs().sum();
        };

        open.push({start, 0.0, heur(start)});
        gscore[toKey(start)] = 0.0;
        const int nbrs[8][2] = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1},
            {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

        bool found = false;
        size_t expands = 0;
        while (!open.empty() && expands < 200000) {
            ++expands;
            Node cur = open.top();
            open.pop();
            if (cur.idx == goal) { found = true; break; }
            const double cg = gscore[toKey(cur.idx)];
            if (cur.g > cg + 1e-9) continue;
            for (auto &n : nbrs) {
                Eigen::Vector3i nxt(cur.idx.x() + n[0], cur.idx.y() + n[1], z0);
                if (!free(nxt)) continue;
                const double step = (n[0] != 0 && n[1] != 0) ? 1.414 : 1.0;
                const double ng = cg + step * res;
                const int64_t k = toKey(nxt);
                auto it = gscore.find(k);
                if (it == gscore.end() || ng < it->second) {
                    gscore[k] = ng;
                    parent[k] = cur.idx;
                    open.push({nxt, ng, ng + heur(nxt)});
                }
            }
        }

        if (!found) {
            p.push_back(s);
            p.push_back(g);
            return (g - s).norm();
        }

        std::vector<Eigen::Vector3i> rev;
        Eigen::Vector3i c = goal;
        while (true) {
            rev.push_back(c);
            if (c == start) break;
            c = parent[toKey(c)];
        }
        p.push_back(s);
        for (auto it = rev.rbegin(); it != rev.rend(); ++it) {
            p.push_back(idxToWorld(*it));
        }
        p.push_back(g);
        double cost = 0.0;
        for (size_t i = 1; i < p.size(); ++i) cost += (p[i] - p[i - 1]).norm();
        return cost;
    }

    inline void convexCover(const std::vector<Eigen::Vector3d> &path,
                            const std::vector<Eigen::Vector3d> &points,
                            const Eigen::Vector3d &lowCorner,
                            const Eigen::Vector3d &highCorner,
                            const double &progress,
                            const double &range,
                            std::vector<Eigen::MatrixX4d> &hpolys,
                            const double eps = 1.0e-6)
    {
        hpolys.clear();
        const int n = static_cast<int>(path.size());
        if (n < 2) {
            return;
        }
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
        bd(0, 0) = 1.0;
        bd(1, 0) = -1.0;
        bd(2, 1) = 1.0;
        bd(3, 1) = -1.0;
        bd(4, 2) = 1.0;
        bd(5, 2) = -1.0;

        Eigen::MatrixX4d hp, gap;
        Eigen::Vector3d a, b = path[0];
        std::vector<Eigen::Vector3d> valid_pc;
        valid_pc.reserve(points.size());
        for (int i = 1; i < n;)
        {
            a = b;
            if ((a - path[i]).norm() > progress)
            {
                b = (path[i] - a).normalized() * progress + a;
            }
            else
            {
                b = path[i];
                i++;
            }

            bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
            bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
            bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
            bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
            bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, highCorner(2));
            bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));

            valid_pc.clear();
            for (const Eigen::Vector3d &pt : points)
            {
                if ((bd.leftCols<3>() * pt + bd.rightCols<1>()).maxCoeff() < 0.0)
                {
                    valid_pc.emplace_back(pt);
                }
            }
            if (valid_pc.empty()) {
                // Degenerate local cloud: use AABB slab as polytope.
                hp = bd;
                hpolys.emplace_back(hp);
                continue;
            }
            Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(
                valid_pc[0].data(), 3, static_cast<int>(valid_pc.size()));

            firi::firi(bd, pc, a, b, hp);

            if (hpolys.size() != 0)
            {
                const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
                if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() +
                             ((hpolys.back() * ah).array() > -eps).cast<int>().sum())
                {
                    firi::firi(bd, pc, a, a, gap, 1);
                    hpolys.emplace_back(gap);
                }
            }

            hpolys.emplace_back(hp);
        }
    }

    inline void shortCut(std::vector<Eigen::MatrixX4d> &hpolys)
    {
        std::vector<Eigen::MatrixX4d> htemp = hpolys;
        if (htemp.size() == 1)
        {
            Eigen::MatrixX4d headPoly = htemp.front();
            htemp.insert(htemp.begin(), headPoly);
        }
        hpolys.clear();

        int M = static_cast<int>(htemp.size());
        bool overlap;
        std::deque<int> idices;
        idices.push_front(M - 1);
        for (int i = M - 1; i >= 0; i--)
        {
            for (int j = 0; j < i; j++)
            {
                if (j < i - 1)
                {
                    overlap = geo_utils::overlap(htemp[i], htemp[j], 0.01);
                }
                else
                {
                    overlap = true;
                }
                if (overlap)
                {
                    idices.push_front(j);
                    i = j + 1;
                    break;
                }
            }
        }
        for (const auto &ele : idices)
        {
            hpolys.push_back(htemp[ele]);
        }
    }

}

#endif
