#pragma once

#include <vector>

class PrevPose {
    public:
        double px_;
        double py_;
        double pz_;
        double qx_;
        double qy_;
        double qz_;
        double qw_;

        void set(double px, double py, double pz, double qx, double qy, double qz, double qw) {
            px_ = px;
            py_ = py;
            pz_ = pz;
            qx_ = qx;
            qy_ = qy;
            qz_ = qz;
            qw_ = qw;
        }
};

void setPrevPose(double& px, double& py, double& pz, double& qx, double& qy, double& qz, double& qw, PrevPose& pp) {
	px = pp.px_;
	py = pp.py_;
	pz = pp.pz_;
	qx = pp.qx_;
	qy = pp.qy_;
	qz = pp.qz_;
	qw = pp.qw_;
}

class VpsResult {
    public:
        double x_;
        double y_;
        double z_;
        VpsResult(double x, double y, double z) {
            x_ = x;
            y_ = y;
            z_ = z;
        }
};

class Averager {
	public:
		Averager() { reset(); }
	private:
        std::vector<VpsResult> pos_;
        const int MAX_COUNT = 4;

	public:
		void add(double x, double y, double z) {
            VpsResult res(x, y, z);
            pos_.push_back(res);
            if (pos_.size() > MAX_COUNT) { 
                pos_.erase(pos_.begin());
            }
		}

        VpsResult avg() {
            double x, y, z;
            x = y = z = 0;
            for (VpsResult r : pos_) {
                x += r.x_;
                y += r.y_;
                z += r.z_;
            }
            x /= pos_.size();
            y /= pos_.size();
            z /= pos_.size();
            VpsResult ret(x, y, z);
            return ret;
        }

		int count() { return pos_.size(); } 
		void reset() { pos_.clear(); }
};

class OdomChecker{

	public:
		OdomChecker(double x, double y, double z) {
			posx_ = x;
			posy_ = y;
			posz_ = z;
		}
	private:
		OdomChecker(){}

	private:
		double posx_;
		double posy_;
		double posz_;

	public:
		double calcDistance(double x, double y, double z) {
			double a = (posx_ - x) * (posx_ - x);
			double b = (posy_ - y) * (posy_ - y);
			return sqrt(a + b);
		}
};