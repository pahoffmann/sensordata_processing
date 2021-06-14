#include "kdTree.h"
#include <string.h>
#include <algorithm>
#include <functional>
#include <iostream>

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
        double *pt = (double*) malloc(n * sizeof(double *));
        pt = pts[i];
        if (pt[0] < xSmall) xSmall = pt[0];
        else if (pt[0] > xBig) xBig = pt[0];
        if (pt[1] < ySmall) ySmall = pt[1];
        else if (pt[1] > yBig) yBig = pt[1];
        if (pt[2] < zSmall) zSmall = pt[2];
        else if (pt[2] > zBig) zBig = pt[2];
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

double *KDtree::FindClosest(double *_p, double maxdist2) {

}

void KDtree::_FindClosest() {
    
}

bool sortPoints(double *p1, double *p2, int axis) {
    return p1[axis] < p2[axis];
}
