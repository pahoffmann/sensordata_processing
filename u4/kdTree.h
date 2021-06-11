#ifndef KDTREE_H
#define KDTREE_H

class KDtree {
    public:
        int npts;       // 0 for inner nodes, others for leaves
        union {
            struct {
                double center[3];
                double dx, dy, dz, r2;
                int splitaxis;
                KDtree *child1, *child2;
            } node;
            struct {
                double *p[8];
            } leaf;
        } ;
        KDtree (double **pts, int n);
        ~KDtree ();
        double *FindClosest (double *_p, double maxdist2);
    private:
        static double *closest;
        static double closest_d2;
        static double *p;
        void _FindClosest();

};

#endif
