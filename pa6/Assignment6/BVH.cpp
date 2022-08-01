#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object *> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);

    // 如果场景内的模型为空则返回，没有模型了哪里还需要去创建加速结构
    if (primitives.empty())
        return;

    // 在此处递归创建BVH的树结构，返回该二叉树的根节点
    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    // 输出构建BVH结构的时间
    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode *BVHAccel::recursiveBuild(std::vector<Object *> objects)
{
    // 目前传进来的是一个Bunny
    BVHBuildNode *node = new BVHBuildNode();

    // 根据object计算包围盒，保证最后每个物体都有自己的一个包围盒
    // 然后再往上求并集组成更大包围盒
    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1)
    {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2)
    {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        // 父节点是其叶子节点包围盒的并集
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else
    {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim)
        {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object *>(beginning, middling);
        auto rightshapes = std::vector<Object *>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray &ray) const
{
    Intersection isect;
    if (!root)
        return isect;

    // 去遍历加速结构BVH，更新ray与空间物体的相交信息
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode *node, const Ray &ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection inter;

    // 遍历至最小包围盒，即当前该包围盒内只有一个物体了，然后获得该object的交点信息
    if (node->left == nullptr && node->right == nullptr)
        return node->object->getIntersection(ray);

    std::array<int, 3> dirIsNeg = {static_cast<int>(ray.direction.x > 0), static_cast<int>(ray.direction.y > 0), static_cast<int>(ray.direction.z > 0)};
    // 判断当前的包围盒是否与ray相交，如果相交了，则继续遍历其子包围盒
    if (node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
    {
        // 遍历直到叶子节点，取前面不会被遮挡的物体
        Intersection leftInter = BVHAccel::getIntersection(node->left, ray);
        Intersection rightInter = BVHAccel::getIntersection(node->right, ray);
        if (leftInter.happened && rightInter.happened)
        {
            inter = leftInter.distance < rightInter.distance ? leftInter : rightInter;
        }
        else
        {
            inter = leftInter.happened ? leftInter : rightInter;
        }
        
    }
    return inter;
}