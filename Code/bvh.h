#pragma once
#include <vector>
#include <memory>
#include <algorithm>
#include "aabb.h"
#include "intersect.h"

struct BVHNode {
    AABB box;
    int left = -1, right = -1;     // child indices; -1 for leaf
    int first = 0, count = 0;      // leaf range into indices[]
    bool isLeaf() const { return left < 0; }
};

class BVH {
public:
    // Build over array of Shape*
    void build(const std::vector<const Shape*>& prims){
        shapes = prims;
        indices.resize(shapes.size());
        for (int i=0;i<(int)indices.size();++i) indices[i]=i;
        nodes.clear();
        nodes.reserve(indices.size()*2);
        buildRecursive(0, (int)indices.size());
    }

    bool intersect(const Ray& ray, Hit& best) const {
        if (nodes.empty()) return false;
        bool hit = false;
        // small stack for traversal
        int stack[64]; int sp = 0;
        stack[sp++] = 0;
        while (sp){
            int ni = stack[--sp];
            const BVHNode& n = nodes[ni];
            float t0,t1;
            if (!intersectAABB(n.box, ray, best.t, t0, t1)) continue;

            if (n.isLeaf()){
                for (int i=0;i<n.count;++i){
                    const Shape* s = shapes[ indices[n.first + i] ];
                    Hit h;
                    if (s->intersect(ray, h) && h.t < best.t){ best = h; hit = true; }
                }
            } else {
                // Traverse nearer child first (optional)
                stack[sp++] = n.left;
                stack[sp++] = n.right;
            }
        }
        return hit;
    }

private:
    std::vector<const Shape*> shapes;
    std::vector<int> indices;      // permutation of shapes
    std::vector<BVHNode> nodes;

    int buildRecursive(int begin, int end){
        BVHNode node;
        // compute bounds of this set
        AABB b; for (int i=begin;i<end;++i) b.expand(shapes[indices[i]]->bounds());
        node.box = b;

        int count = end - begin;
        int nodeIndex = (int)nodes.size();
        nodes.push_back(node);

        // Leaf?
        const int leafMax = 4;
        if (count <= leafMax){
            nodes[nodeIndex].first = begin;
            nodes[nodeIndex].count = count;
            return nodeIndex;
        }

        // Choose split axis = longest extent
        int axis = b.maxExtent();

        // Compute primitive centers along axis
        auto centerCoord = [&](int idx)->float{
            AABB pb = shapes[idx]->bounds();
            Vec3 c = (pb.min + pb.max) * 0.5f;
            return (axis==0? c.x : (axis==1? c.y : c.z));
        };

        // Partition by median along 'axis'
        int mid = (begin + end) / 2;
        std::nth_element(indices.begin()+begin, indices.begin()+mid, indices.begin()+end,
            [&](int a, int c){
                return centerCoord(a) < centerCoord(c);
            });

        // Build children
        int left  = buildRecursive(begin, mid);
        int right = buildRecursive(mid, end);
        nodes[nodeIndex].left = left;
        nodes[nodeIndex].right = right;
        return nodeIndex;
    }
};
