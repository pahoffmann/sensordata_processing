#include "kdTree.h"

double KDtree::closest_d2;
double *KDtree::closest;
double **KDtree::kNearest;
double *KDtree::p;

// squared distance
double Dist2(double *p1, double *p2) {
    return pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2) + pow(p1[2] - p2[2], 2);
}

KDtree::KDtree(double **pts, int n) {

    if (n <= 8) {
        npts = n;
        memcpy(leaf.p, pts, n* sizeof(double *));
        return;
    }
    npts = 0;        // Baum enthält keine Punkte, sondern Unterbäume

    /*
    // sort the points along given axis to find median
    // std::sort(pts, pts + n, std::bind(sortPoints, _1, _2, node.splitaxis);
    // sort the points
    std::sort(pts, pts + n);
    node.center = pts[n / 2];
    */

    // bounding box
    // initial values
    double xSmall, xBig, ySmall, yBig, zSmall, zBig;
    xSmall = pts[0][0];
    xBig = pts[0][0];
    ySmall = pts[0][1];
    yBig = pts[0][1];
    zSmall = pts[0][2];
    zBig = pts[0][2];

    for (int i = 0; i < n; i++) {
        //double *pt = (double*) malloc(n * sizeof(double *));
        double *pt = pts[i];
        if (pt[0] < xSmall) xSmall = pt[0];
        else if (pt[0] > xBig) xBig = pt[0];
        if (pt[1] < ySmall) ySmall = pt[1];
        else if (pt[1] > yBig) yBig = pt[1];
        if (pt[2] < zSmall) zSmall = pt[2];
        else if (pt[2] > zBig) zBig = pt[2];
        //free(pt);
    }

    node.dx = xBig - xSmall;
    node.dy = yBig - ySmall;
    node.dz = zBig - zSmall;
    node.center[0] = xBig - node.dx / 2;
    node.center[1] = yBig - node.dy / 2;
    node.center[2] = zBig - node.dz / 2;

    /*
    printf("dx: %f, dy: %f, dz: %f, cx: %f, cy: %f, cz: %f\n", 
        node.dx, node.dy, node.dz, node.center[0], node.center[1], node.center[2]);
    */

    // split on longest axis
    if (node.dx > node.dy)
        if (node.dx > node.dz) node.splitaxis = 0;
        else node.splitaxis = 2;
    else 
        if (node.dy > node.dz) node.splitaxis = 1;
        else node.splitaxis = 2;

    double splitval = node.center[node.splitaxis];
    double **left, **right;

    // check how many elements for left and right tree
    // to allocate enough memory
    // left: <=, right: >
    int leftCnt = 0, rightCnt = 0;
    for (int i = 0; i < n; i++) {
        if (pts[i][node.splitaxis] <= splitval)
            leftCnt++;
        else 
            rightCnt++;
    }

    left = (double **) malloc(leftCnt * sizeof(double*));
    /*for (int i = 0; i < leftCnt; i++) {
        left[i] = (double*) malloc(3 * sizeof(double));
    }*/
    right = (double **) malloc(rightCnt * sizeof(double*));
    /*for (int i = 0; i < rightCnt; i++) {
        right[i] = (double*) malloc(3 * sizeof(double));
    }*/
    
    /*
    // check left, right, n values
    printf("leftCnt: %d, rightCnt: %d, n: %d\n", leftCnt, rightCnt, n);
    if (node.splitaxis == 0) {
        printf("Axis: %d, xSmall: %f, xBig: %f, splitval: %f\n", node.splitaxis, xSmall, xBig, splitval);
    } else if (node.splitaxis == 1) {
        printf("Axis: %d, ySmall: %f, yBig: %f, splitval: %f\n", node.splitaxis, ySmall, yBig, splitval);
    } else if (node.splitaxis == 2) {
        printf("Axis: %d, zSmall: %f, zBig: %f, splitval: %f\n", node.splitaxis, zSmall, zBig, splitval);
    }*/    

    // fill the trees with values
    leftCnt = 0; rightCnt = 0;
    for (int i = 0; i < n; i++) {
        if (pts[i][node.splitaxis] <= splitval) {
            // copy data from pts to left
            //memcpy(left[leftCnt++], pts[i], sizeof(double *));
            left[leftCnt++] = pts[i];
        }
        else {
            //memcpy(right[rightCnt++], pts[i], sizeof(double *));
            right[rightCnt++] = pts[i];

        }
    }

    node.child1 = new KDtree(left, leftCnt);
    node.child2 = new KDtree(right, rightCnt);

}

KDtree::~KDtree() {

}

double **KDtree::kNearestNeighbors(double *_p, double maxdist2, int k, int &num_neighbors)
{
    std::vector<std::pair<double*, double>> nearestVec;
    kNearest = new double*[k];
    closest = NULL; closest_d2 = maxdist2; p = _p;
    _kNearestNeighbors(nearestVec);

    //std::cout << "num possible vertices: " << nearestVec.size() << std::endl;

    //sort found vertices
    std::sort(nearestVec.begin(), nearestVec.end(), 
                [](std::pair<double*, double> a, std::pair<double*, double> b)
                {
                    return a.second < b.second;
                });

    for(int i = 0; i < std::min((int)nearestVec.size(), k); i++)
    {
        kNearest[i] = new double[3];
        kNearest[i][0] = nearestVec[i].first[0];
        kNearest[i][1] = nearestVec[i].first[1];
        kNearest[i][2] = nearestVec[i].first[2];
    }

    num_neighbors = std::min((int)nearestVec.size(), k);

    return kNearest;

}


void KDtree::_kNearestNeighbors( std::vector<std::pair<double*, double>>& nearest)
{
    if (npts) {
        for (int i = 0; i < npts; i++) {
            double myd2 = Dist2(p, leaf.p[i]);
            if (myd2 < closest_d2) {
                nearest.push_back(std::make_pair(leaf.p[i], myd2));
            }
        }
        return;
    }

    // define criteria wether to dig deep into other subtrees..
    // our criteria is obvious, but may not be the fastest, as there are special cases, when subtrees are visited unnecessarily
    // we just assume, that the distances between two of the x, y and z components from the point need to be individually smaller than the radius of the search
    // but if we leave out subtrees we leave them out with our well earned right #2ndAmendment
    // for the left and right subtree individually

    if(node.child1->npts > 8)
    {
        double x1 = node.child1->node.center[0] - (node.child1->node.dx/2);
        double x2 = node.child1->node.center[0] + (node.child1->node.dx/2);
        double y1 = node.child1->node.center[1] - (node.child1->node.dy/2);
        double y2 = node.child1->node.center[1] + (node.child1->node.dy/2);
        double z1 = node.child1->node.center[2] - (node.child1->node.dz/2);
        double z2 = node.child1->node.center[2] + (node.child1->node.dz/2);

        int cond_counter = 0;

        if(closest_d2 > pow(abs(p[0] - x1), 2) || closest_d2 > pow(abs(p[0] - x2), 2))
        {
            cond_counter++;
        }

        if(closest_d2 > pow(abs(p[1] - y1), 2) || closest_d2 > pow(abs(p[1] - y2), 2))
        {
            cond_counter++;
        }

        if(closest_d2 > pow(abs(p[2] - z1), 2) || closest_d2 > pow(abs(p[2] - z2), 2))
        {
            cond_counter++;
        }

        if(cond_counter >= 2)
        {
            node.child1->_kNearestNeighbors(nearest);
        }
    } 
    else {
        node.child1->_kNearestNeighbors(nearest);
    }   

    if(node.child2->npts > 8)
    {
        double x1 = node.child2->node.center[0] - (node.child2->node.dx/2);
        double x2 = node.child2->node.center[0] + (node.child2->node.dx/2);
        double y1 = node.child2->node.center[1] - (node.child2->node.dy/2);
        double y2 = node.child2->node.center[1] + (node.child2->node.dy/2);
        double z1 = node.child2->node.center[2] - (node.child2->node.dz/2);
        double z2 = node.child2->node.center[2] + (node.child2->node.dz/2);

        int cond_counter = 0;

        if(closest_d2 > pow(abs(p[0] - x1), 2) || closest_d2 > pow(abs(p[0] - x2), 2))
        {
            cond_counter++;
        }

        if(closest_d2 > pow(abs(p[1] - y1), 2) || closest_d2 > pow(abs(p[1] - y2), 2))
        {
            cond_counter++;
        }

        if(closest_d2 > pow(abs(p[2] - z1), 2) || closest_d2 > pow(abs(p[2] - z2), 2))
        {
            cond_counter++;
        }

        if(cond_counter >= 2)
        {
            node.child2->_kNearestNeighbors(nearest);
        }
    } 
    else {
        node.child2->_kNearestNeighbors(nearest);
    }  

    return;  
}

double *KDtree::FindClosest(double *_p, double maxdist2) {
    closest = NULL; closest_d2 = maxdist2; p = _p;
    _FindClosest();
    return closest;
}

void KDtree::_FindClosest() {
    if (npts) {
        for (int i = 0; i < npts; i++) {
            double myd2 = Dist2(p, leaf.p[i]);
            if (myd2 < closest_d2) {
                closest_d2 = myd2;
                closest = leaf.p[i];
            }
        }
        return;
    }

    // maximale Distanz des Punktes zur aktuellen Bounding Box
    double approx_dist_bbox = std::max(std::max(fabs(p[0] - node.center[0]) - node.dx,
                                                fabs(p[1] - node.center[1]) - node.dy),
                                            fabs(p[2] - node.center[2]) - node.dz);
    // wenn bounding box weiter weg als Distanz zu bisher gefundenem nächsten Punkt
    if (approx_dist_bbox >= 0 && pow(approx_dist_bbox, 2) >= closest_d2) 
        return;
    double myd = node.center[node.splitaxis] - p[node.splitaxis];
    // Gehe zum Weitersuchen in die Unterbäume
    if (myd >= 0.0f) {
        node.child1->_FindClosest();
        // if point near other child tree, then also search in this child tree
        if (pow(myd, 2) < closest_d2) node.child2->_FindClosest();
    } else {
        node.child2->_FindClosest();
        if (pow(myd, 2) < closest_d2) node.child1->_FindClosest();
    }
}

bool sortPoints(double *p1, double *p2, int axis) {
    return p1[axis] < p2[axis];
}
