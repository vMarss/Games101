#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

// BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
// {
//     /**
//      * ======================
//      * == 递归构建BVH加速结构 ==
//      * ======================
//      * 
//      *  objects: 图元信息数组
//      * 
//     **/
//     BVHBuildNode* node = new BVHBuildNode();

//     // 计算每个图元的BVH节点
//     // 所以说.obj文件提供的是一堆图元(三角片片)顶点等的信息
//     // Compute bounds of all primitives in BVH node
    
//     // 用于SAH的计算
//     // Bounds3 bounds;
//     // for (int i = 0; i < objects.size(); ++i)
//     //     bounds = Union(bounds, objects[i]->getBounds())


//     if (objects.size() == 1) {
//         // Create leaf _BVHBuildNode_
//         // 遍历至最后一个图元，是作为叶子节点，叶子节点需要保存图元的信息，即object
//         node->bounds = objects[0]->getBounds();
//         node->object = objects[0];
//         node->left = nullptr;
//         node->right = nullptr;
//         return node;
//     }
//     else if (objects.size() == 2) {
//         // 非叶子节点只需要保存一个bound边界信息
//         node->left = recursiveBuild(std::vector<Object*>{objects[0]});
//         node->right = recursiveBuild(std::vector<Object*>{objects[1]});

//         node->bounds = Union(node->left->bounds, node->right->bounds);
//         return node;
//     }
//     else {
//         //////////////////////////////////////////// 计算图元质心的边界，选择分割的坐标轴 ////////////////////////////////////////////////////

//         // 这里排序的目的是？
//         // 是为了判断下一步究竟是根据x轴来二等分还是根据y轴或z轴
//         // 那判断的依序又是什么呢

//         // 1. 求剩下所有图元的重心bounds并集
//         Bounds3 centroidBounds;
//         for (int i = 0; i < objects.size(); ++i)
//             // p.s. 每次递归都会对剩下的一半求并集，感觉是对之前的计算重复了，是否可用动态规划算法优化？
//             centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());


//         std::vector<Object*> leftshapes;
//         std::vector<Object*> rightshapes;

//         if (splitMethod == SplitMethod::NAIVE)
//         {
//             int dim = centroidBounds.maxExtent();
//             // 针对最大跨度的轴进行排序，但为何需要取质心??
//             // 为了方便排序？不然需要考虑取包围盒的min还是max来排序的问题，取重心坐标排序比较平均
//             // 答：这是一个通常的图元划分方式，目的就是将图元划分为两部分，所以求图元在那个轴上的覆盖范围最大，所以取质心来求在哪个轴上覆盖最大
//             switch (dim) {
//             case 0:
//                 std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
//                     return f1->getBounds().Centroid().x <
//                         f2->getBounds().Centroid().x;
//                 });
//                 break;
//             case 1:
//                 std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
//                     return f1->getBounds().Centroid().y <
//                         f2->getBounds().Centroid().y;
//                 });
//                 break;
//             case 2:
//                 std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
//                     return f1->getBounds().Centroid().z <
//                         f2->getBounds().Centroid().z;
//                 });
//                 break;
//             }

//             // 根据object的中点划分，那必须得保证object是有序的
//             // 比较暴力的空间划分方式，直接以跨度大的轴对半分
//             auto beginning = objects.begin();
//             auto middling = objects.begin() + (objects.size() / 2);
//             auto ending = objects.end();

//             leftshapes = std::vector<Object*>(beginning, middling);
//             rightshapes = std::vector<Object*>(middling, ending);
//         }
//         else {
//             // 获取当前包围盒的最大表面积
//             float fArea = centroidBounds.SurfaceArea();

//             int minCostCoor = 0;
//             int minCostIndex = 0;

//             float minCost = std::numeric_limits<float>::infinity();
//             std::map<int, std::map<int ,int>> indexMap;

//             // 三个维度，x，y，z
//             for (int i = 0; i < 3; i++)
//             {   
//                 // 相比于遍历所有可能的排列情况，将当前的面积分为一定数量的桶来计算开销得到的结果和遍历所有情况差不多
//                 int bucketCount = 12;
//                 std::vector<Bounds3> boundsBuckets;
//                 std::vector<int> countBucket;
//                 for (int j = 0; j < bucketCount; j++)
//                 {
//                     boundsBuckets.push_back(Bounds3());
//                     countBucket.push_back(0);
//                 }

//                 std::map<int, int> objMap;

//                 for (int j = 0; j < objects.size(); j++)
//                 {
//                     int bid = bucketCount * (centroidBounds.Offset(objects[j]->getBounds().Centroid()))[i];

//                     if (bid > bucketCount - 1)
//                     {
//                         bid = bucketCount - 1;
//                     }
//                     Bounds3 b = boundsBuckets[bid];
//                     b = Union(b, objects[j]->getBounds().Centroid());
//                     boundsBuckets[bid] = b;
//                     countBucket[bid] = countBucket[bid] + 1;
//                     objMap.insert(std::make_pair(j, bid));
//                 }

//                 indexMap.insert(std::make_pair(i, objMap));

//                 // 对于每一个划分，计算其花费，方法是对于桶中的每一个面积，计算其花费
//                 for (int j = 0; j < boundsBuckets.size(); j++)
//                 {
//                     Bounds3 A;
//                     Bounds3 B;
//                     int countA = 0;
//                     int countB = 0;
//                     for (int k = 0; k < j; k++)
//                     {
//                         A = Union(A, boundsBuckets[k]);
//                         countA += countBucket[k];
//                     }

//                     for (int k = 0; k < boundsBuckets.size(); k++)
//                     {
//                         B = Union(B, boundsBuckets[k]);
//                         countB += countBucket[k];
//                     }

//                     // 计算花费
//                     float cost = 1 + (countA * A.SurfaceArea() + countB * B.SurfaceArea()) / fArea;

//                     if (cost < minCost)
//                     {
//                         minCost = cost;
//                         minCostIndex = j;
//                         minCostCoor = i;
//                     }
//                 }
//             }

//             for (int i = 0; i < objects.size(); i++)
//             {
//                 if (indexMap[minCostCoor][i] < minCostIndex)
//                 {
//                     if (indexMap[minCostCoor][i] < minCostIndex)
//                     {
//                         leftshapes.push_back(objects[i]);
//                     }
//                     else
//                     {
//                         rightshapes.push_back(objects[i]);
//                     }
//                 }
                
//             }
//         }

//         node->left = recursiveBuild(leftshapes);
//         node->right = recursiveBuild(rightshapes);

//         node->bounds = Union(node->left->bounds, node->right->bounds);
//     }

//     return node;
// }

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else
    {
        Bounds3 centroidBounds;
        //算出最大的包围盒（通用的，因为两个方法都要用到）
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());

        std::vector<Object*> leftshapes;
        std::vector<Object*> rightshapes;

            switch (splitMethod)//这里注意了在BVH.h里面有个枚举类，构造函数中的初始将决定是普通方法，还是SAH
            {
            case SplitMethod::NAIVE:
            {
                int dim = centroidBounds.maxExtent();//算出最大的跨度对应的值，x为0，y为1，z为2
                int index = objects.size() / 2;
                switch (dim)
                //排序，针对最大的跨度排序
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
                auto middling = objects.begin() + index;
                auto ending = objects.end();
                 //递归算法，枢轴是中间元素。
                leftshapes = std::vector<Object *>(beginning, middling);
                rightshapes = std::vector<Object *>(middling, ending);
            }
            break;
            case SplitMethod::SAH:
            {
                float nArea = centroidBounds.SurfaceArea();//算出最大的

                int minCostCoor = 0;
                int mincostIndex = 0;
                float minCost = std::numeric_limits<float>::infinity();
                std::map<int, std::map<int, int>> indexMap;
                //indexmap用于记录x，y，z（前一个int代表x，y，z，后一个map代表那个轴对应的map）
                //遍历x，y，z的划分
                for(int i = 0; i < 3; i++)
                {
                    int bucketCount = 12;//桶的个数，这里定了12个桶，就是在某一个轴上面划分了12个区域
                    std::vector<Bounds3> boundsBuckets;
                    std::vector<int> countBucket;
                    for(int j = 0; j < bucketCount; j++)
                    {
                        boundsBuckets.push_back(Bounds3());
                        countBucket.push_back(0);
                    }

                    std::map<int, int> objMap;

                    for(int j = 0; j < objects.size(); j++)
                    {   
                        Vector3f tmp = centroidBounds.Offset(objects[j]->getBounds().Centroid());
                        int bid = bucketCount * tmp[0];
                        
                        if(i == 1)
                        {
                            int bid =  bucketCount * tmp.y;
                        }
                        else if(i == 2)
                        {
                            int bid =  bucketCount * tmp.z;
                        }

                        if(bid > bucketCount - 1)//实质是可以划分13个区域的，将最后一个区域合并。
                        {
                            bid = bucketCount - 1;
                        }
                        Bounds3 b = boundsBuckets[bid];
                        b = Union(b, objects[j]->getBounds().Centroid());
                        boundsBuckets[bid] = b;
                        countBucket[bid] = countBucket[bid] + 1;
                        objMap.insert(std::make_pair(j, bid));
                    }

                    indexMap.insert(std::make_pair(i, objMap));
                    //对于每一个划分，计算他所对应的花费，方法是对于桶中的每一个面积，计算他的花费，最后进行计算
                    //找出这个划分。
                    for(int j = 1; j < boundsBuckets.size(); j++)
                    {
                        Bounds3 A;
                        Bounds3 B;
                        int countA = 0;
                        int countB = 0;
                        for(int k = 0; k < j; k++)
                        {
                            A = Union(A, boundsBuckets[k]);
                            countA += countBucket[k];
                        }

                        for(int k = j; k < boundsBuckets.size(); k++)
                        {
                            B = Union(B, boundsBuckets[k]);
                            countB += countBucket[k];
                        }

                        float cost = 1 + (countA * A.SurfaceArea() + countB * B.SurfaceArea()) / nArea;//计算花费
                        //找出这个花费。
                        if(cost < minCost)
                        {
                            minCost = cost;
                            mincostIndex = j;
                            minCostCoor = i;
                        }
                    }
                }
                //加入左右数组，这里很重要，具体还是看那篇博客
                for(int i = 0; i < objects.size(); i++)
                {
                    if(indexMap[minCostCoor][i] < mincostIndex)
                    {
                        leftshapes.push_back(objects[i]);
                    }
                    else
                    {
                        rightshapes.push_back(objects[i]);
                    }
                }
            }
            break;
            dafault:
            break;
        }

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));
        //递归计算，同普通方法
        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    // std::cout << " root = " << isect.happened << std::endl;
    isect = BVHAccel::getIntersection(root, ray);
    // std::cout << " isect = " << isect.happened << std::endl;
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // 此处在递归求交集，但是完全可以省去这个递归遍历树的过程，所以可以将树结构以数组形式存储(TODO 如何构建这个一维的树，记得以前构造堆有看过，忘记了)
    // std::cout << " xxxxxxxxxxxxxxxxxxx getIntersection " << node->left << " : " << node->right << std::endl;
    
    // TODO Traverse the BVH to find intersection
    Intersection inter;

    if (node == nullptr)
        return inter;

    // 遍历至最小包围盒，即当前该包围盒内只有一个物体了，然后获得该object的交点信息
    if (node->left == nullptr && node->right == nullptr)
    {
        // std::cout << " xxxxxxxxxxxxxxxxxxx get inter " << std::endl;
        return node->object->getIntersection(ray);
    }

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