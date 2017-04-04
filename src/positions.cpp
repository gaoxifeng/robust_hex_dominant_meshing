#include "hierarchy.h"
#include "positions.h"
#include "timer.h"

void MultiResolutionHierarchy::smoothPositionsTri(uint32_t l, bool alignment, bool randomization, bool extrinsic) {
    const SMatrix &L = mL[l];
    const MatrixXf &V = mV[l], &N = mN[l], &Q = mQ[l];
    MatrixXf &O = mO[l];

    Timer<> timer;
   // timer.beginStage("Smoothing orientations at level " + std::to_string(l));

    double error = 0;
    int nLinks = 0;
    MatrixXf O_new(O.rows(), O.cols());
    tbb::spin_mutex mutex;

    tbb::parallel_for(
        tbb::blocked_range<uint32_t>(0u, (uint32_t) L.outerSize(), GRAIN_SIZE),
        [&](const tbb::blocked_range<uint32_t> &range) {
            std::vector<std::pair<uint32_t, Float>> neighbors;
            double errorLocal = 0;
            int nLinksLocal = 0;
            for (uint32_t k = range.begin(); k != range.end(); ++k) {
                SMatrix::InnerIterator it(L, k);

                uint32_t i = it.row();

                const Vector3f q_i = Q.col(i);
                const Vector3f n_i = N.col(i), v_i = V.col(i);
                Vector3f o_i = O.col(i);

                neighbors.clear();
                for (; it; ++it) {
                    uint32_t j = it.col();
                    if (i == j)
                        continue;
                    neighbors.push_back(std::make_pair(j, it.value()));
                }

                if (randomization && neighbors.size() > 0)
                    pcg32(mPositionIterations, k)
                        .shuffle(neighbors.begin(), neighbors.end());

                Float weightSum = 0.f;
                for (auto n : neighbors) {
                    uint32_t j = n.first;
                    Float value = n.second;

                    const Vector3f q_j = Q.col(j), v_j = V.col(j), n_j = N.col(j);
                    Vector3f o_j = O.col(j);

                    if (extrinsic) {
                        errorLocal += (O.col(i) -
                             findClosestPairExtrinsic(O.col(i), q_i, n_i, v_i, o_j, q_j, n_j,
                                             v_j, mScale, mInvScale)).norm();
                        o_j = findClosestPairExtrinsic(o_i, q_i, n_i, v_i, o_j, q_j, n_j,
                                              v_j, mScale, mInvScale);
                    } else {
                        errorLocal += (O.col(i) -
                             findClosestPair(O.col(i), q_i, n_i, v_i, o_j, q_j, n_j,
                                             v_j, mScale, mInvScale).first).norm();
                        o_j = findClosestPair(o_i, q_i, n_i, v_i, o_j, q_j, n_j,
                                              v_j, mScale, mInvScale).first;
                    }

                    o_i = value * o_j + weightSum * o_i;
                    weightSum += value;
                    o_i /= weightSum;
                    nLinksLocal++;
                }

                o_i = findClosest(o_i, q_i, n_i, v_i, mScale, mInvScale);
                o_i -= n_i.dot(o_i - v_i) * n_i;

				//if(l ==0 && nV_boundary_flag[l][i])
				if (nV_boundary_flag[l][i])
					o_i = q_i.dot(o_i - v_i) * q_i + v_i;
                
				O_new.col(i) = o_i;
            }
            tbb::spin_mutex::scoped_lock guard(mutex);
            error += errorLocal;
            nLinks += nLinksLocal;
        }
    );
    //timer.endStage("E = " + std::to_string(error / nLinks));
    mOrientationIterations++;
    O = std::move(O_new);
}

void MultiResolutionHierarchy::smoothPositionsTriCombed() {
    auto const &Q = mQ_combed, &N = mN[0], &V = mV[0];
    auto const &Oi = mO_combed;
    auto const &F = mF;
    auto &O = mO[0];

    MatrixXf O_new;
    VectorXi count;
    O_new.setZero(O.rows(), O.cols());
    count.setZero(O.cols());

    for (uint32_t f = 0; f < mF.cols(); ++f) {
        for (uint32_t i =0; i<3; ++i) {
            uint32_t v0 = F(i, f), v1 = F((i + 1) % 3, f);
            typedef Eigen::Matrix<Float, 3, 2> Matrix;

            const Vector3f &q0 = Q.col(3 * f + i);
            const Vector3f &q1_ = Q.col(3 * f + (i + 1) % 3);
            const Vector2i &t0 = Oi.col(3 * f + i);
            const Vector3f &o1_ = O.col(v1);
            const Vector3f &n0 = N.col(v0), &n1 = N.col(v1);
            const Vector3f &p0 = V.col(v0), &p1 = V.col(v1);

            Vector3f q1 = rotateVectorIntoPlane(q1_, n1, n0);
            Vector3f qn = (q0 + q1).normalized();
            Vector3f middle = middle_point(p0, n0, p1, n1);
            Vector3f o1 = rotateVectorIntoPlane(o1_ - middle, n1, n0) + middle;
            Matrix M = (Matrix() << qn, n0.cross(qn)).finished();
            o1 += (M * t0.cast<Float>()) * mScale;
            o1 -= n0.dot(o1 - p0) * n0;

            O_new.col(v0) += o1;
            count[v0]++;
        }
    }
    for (int i=0; i<O.cols(); ++i) {
        if (count[i] > 0)
            O_new.col(i) /= count[i];
    }
    O = std::move(O_new);
}

void MultiResolutionHierarchy::smoothPositionsTet(uint32_t l, bool alignment, bool randomization) {
    const SMatrix &L = mL[l];
    const MatrixXf &V = mV[l], &N = mN[l], &Q = mQ[l], &C = mC[l];
    MatrixXf &O = mO[l];

    Timer<> timer;
    //timer.beginStage("Smoothing orientations at level " + std::to_string(l));

    double error = 0;
    int nLinks = 0;
    MatrixXf O_new(O.rows(), O.cols());
    tbb::spin_mutex mutex;

#if 1
    tbb::parallel_for(
        tbb::blocked_range<uint32_t>(0u, (uint32_t) L.outerSize(), GRAIN_SIZE),
        [&](const tbb::blocked_range<uint32_t> &range) {
            std::vector<std::pair<uint32_t, Float>> neighbors;
            double errorLocal = 0;
            int nLinksLocal = 0;
            for (uint32_t k = range.begin(); k != range.end(); ++k) {
#endif
				//std::vector<std::pair<uint32_t, Float>> neighbors;
				//double errorLocal = 0;
				//int nLinksLocal = 0;
				//for (uint32_t k = 0; k <L.outerSize(); ++k) {
				//	
					SMatrix::InnerIterator it(L, k);

                uint32_t i = it.row();

                const Quaternion q_i = Q.col(i);
                const Vector3f n_i = N.col(i), v_i = V.col(i), c_i = C.col(i);
                Vector3f o_i = O.col(i);

                neighbors.clear();
                for (; it; ++it) {
                    uint32_t j = it.col();
                    if (i == j)
                        continue;
                    neighbors.push_back(std::make_pair(j, it.value()));
                }

                if (randomization && neighbors.size() > 0)
                    pcg32(mPositionIterations, k)
                        .shuffle(neighbors.begin(), neighbors.end());

                Float weightSum = 0.f;
                for (auto n : neighbors) {
                    uint32_t j = n.first;
                    Float value = n.second;

                    const Quaternion q_j = Q.col(j);
                    Vector3f o_j = O.col(j);

                    errorLocal += (O.col(i) - findClosestPair(O.col(i), q_i, o_j, q_j, mScale, mInvScale).first).norm();
                    nLinksLocal++;

                    o_j = findClosestPair(o_i, q_i, o_j, q_j, mScale, mInvScale).first;
                    o_i = value * o_j + weightSum * o_i;
                    weightSum += value;
                    o_i /= weightSum;

                    if (alignment && n_i != Vector3f::Zero()) {
                        Float dp = n_i.dot(c_i - o_i) * mInvScale;
                        o_i += (dp - std::round(dp)) * n_i * mScale;  
					}
					//if (alignment && n_i != Vector3f::Zero())
					//	o_i -= n_i.dot(o_i - v_i) * n_i;
                }

				o_i = findClosest(o_i, q_i, v_i, mScale, mInvScale);

				//if (l == 200000 && n_i != Vector3f::Zero()) {
				//	Vector3d v = o_i.cast<double>();
				//	//cout << "phone" << endl;
				//	Vector3d interpolP, interpolN;
				//	vector<uint32_t>  tids = vnfs[i];

				//	//tbb::spin_mutex::scoped_lock guard(mutex);
				//	//if (phong_projection(tids, vnfs, v, interpolP, interpolN)) {
				//		//o_i = interpolP.cast<Float>();
				//		//cout << "projected" << endl;
				//	//}

				//}
				//if (n_i != Vector3f::Zero())
				//	o_i -= n_i.dot(o_i - v_i) * n_i;
				O_new.col(i) = o_i;

			}
            tbb::spin_mutex::scoped_lock guard(mutex);
            error += errorLocal;
            nLinks += nLinksLocal;

#if 1
        }
    );
#endif

    //timer.endStage("E = " + std::to_string(error / nLinks));
    mOrientationIterations++;
    O = std::move(O_new);
}


void MultiResolutionHierarchy::projectBack3D() {
	//cout << "project back" << endl;
	//vector<vector<uint32_t>> vnfs(mO[0].cols());
	//for (uint32_t i = 0; i < mF.cols(); ++i) for (uint32_t j = 0; j < 3; j++) vnfs[mF(j, i)].push_back(i);

	int count = 0;
	for (uint32_t i = 0; i < vnfs.size(); i++) {
		if (!vnfs[i].size()) continue;

		Vector3d v = mO[0].col(i).cast<double>();

		uint32_t tid = 0;
		double min_dis = (v - mCF.col(0).cast<double>()).norm();
		for (uint32_t j = 1; j < mCF.cols(); j++) {
			if (min_dis >(v - mCF.col(j).cast<double>()).norm()) {
				tid = j;
				min_dis = (v - mCF.col(j).cast<double>()).norm();
			}
		}
		uint32_t tid_temp = tid;
		vector<uint32_t> tids; tids.push_back(tid_temp);

		vector<uint32_t> tids_;
		for (uint32_t Iter = 0; Iter < 1; Iter++) {
			for (uint32_t j = 0; j < tids.size(); j++) {
				for (uint32_t k = 0; k < 3; k++) {
					uint32_t vid = mF(k, tids[j]);
					for (auto ntid : vnfs[vid]) tids_.push_back(ntid);
				}
			}
			tids.insert(tids.end(), tids_.begin(), tids_.end()); tids_.clear();
		}
		sort(tids.begin(), tids.end()); tids.erase(unique(tids.begin(), tids.end()), tids.end());

		Vector3d interpolP, interpolN;

		if (phong_projection(tids, vnfs, v, interpolP, interpolN, false)) {
			mO[0].col(i) = interpolP.cast<float>();
			//cout << "projected" << endl;
			count++;
			cout << "i " << i << endl;
		}
	}
	cout << "projected #"<<count << endl;

}
bool MultiResolutionHierarchy::phong_projection(vector<uint32_t> tids, vector<vector<uint32_t>> &vnfs, Vector3d &v, Vector3d &interpolP, Vector3d &interpolN, bool global) {
	vector<bool> t_flag(mF.cols(), false);
	cout << "project" << endl;
	vector<uint32_t> ts;
	bool found = false;
	for (auto id : tids) { t_flag[id] = true; ts.push_back(id); }
	while (ts.size()) {
		tids.clear();
		vector<Vector3d> pvs, pns;
		vector<Vector2d> uvs; vector<pair<double, uint32_t>> dis_ids;
		for (uint32_t j = 0; j < ts.size(); j++) {
			vector<Vector3d> tri_vs(3), vs_normals(3);
			for (uint32_t k = 0; k < 3; k++) {
				uint32_t vid = mF(k, ts[j]);
				tri_vs[k] = mV[0].col(vid).cast<double>();
				vs_normals[k] = mNF.col(vid).cast<double>();
			}
			Vector2d uv;
			projectPointOnTriangle(tri_vs, vs_normals, v, uv, interpolP, interpolN);

			if ((uv.x() >= 0.0 || num_equal(uv.x(), 0.0, Precision_Pro)) && (uv.y() >= 0.0 || num_equal(uv.y(), 0.0, Precision_Pro)) &&
				(1 - uv.x() - uv.y() >= 0.0 || num_equal(1 - uv.x() - uv.y(), 0.0, Precision_Pro))) {
				dis_ids.push_back(make_pair((v - interpolP).norm(), tids.size()));
				pvs.push_back(interpolP); pns.push_back(interpolN); uvs.push_back(uv);
				tids.push_back(ts[j]);
			}
		}
		sort(dis_ids.begin(), dis_ids.end());

		if (dis_ids.size()) {
			found = true;
			uint32_t cloestid = dis_ids[0].second;
			interpolP = pvs[cloestid];
			interpolN = pns[cloestid];
			return true;
		}
		if (!global) {
			cout << "project end" << endl;
			return false;
		}

		vector<uint32_t> ts_;
		for (uint32_t j = 0; j < ts.size(); j++) {
			for (uint32_t k = 0; k < 3; k++) {
				uint32_t vid = mF(k, ts[j]);
				for (auto ntid : vnfs[vid]) {
					if (t_flag[ntid]) continue;
					t_flag[ntid] = true;
					ts_.push_back(ntid);
				}
			}
		}
		ts.clear();
		ts.swap(ts_);

		cout << "continue the loop" << endl;
	}
	return false;
}
void projectPointOnQuad(const vector<Vector3d>& quad_vs, vector<Vector3d> & vs_normals, const Vector3d& p, Vector2d& uv, Vector3d& interpolP, Vector3d& interpolN)
{
	Eigen::Matrix<double, 3, 2> jacobian;

	uv = Vector2d::Constant(0.5f);

	//Newton iteration for projection on quad

	//objective function:
	// F(u, v) = (p - interpolP) x interpolN = 0
	Vector3d F;

	for (int i = 0; i < 4; ++i)
	{
		interpolP = bilinear(quad_vs[0], quad_vs[1], quad_vs[2], quad_vs[3], uv);
		interpolN = bilinear(vs_normals[0], vs_normals[1], vs_normals[2], vs_normals[3], uv);

		Vector3d dPdu = (1 - uv.y()) * quad_vs[1] + uv.y() * quad_vs[2] - ((1 - uv.y()) * quad_vs[0] + uv.y() * quad_vs[3]);
		Vector3d dPdv = (1 - uv.x()) *quad_vs[3] + uv.x() * quad_vs[2] - ((1 - uv.x()) * quad_vs[0] + uv.x() * quad_vs[1]);
		Vector3d dNdu = (1 - uv.y()) * vs_normals[1] + uv.y() * vs_normals[2] - ((1 - uv.y()) * vs_normals[0] + uv.y() * vs_normals[3]);
		Vector3d dNdv = (1 - uv.x()) * vs_normals[3] + uv.x() * vs_normals[2] - ((1 - uv.x()) * vs_normals[0] + uv.x() * vs_normals[1]);

		F = (p - interpolP).cross(interpolN);
		Vector3d dFdu = (-dPdu).cross(interpolN) + (p - interpolP).cross(dNdu);
		Vector3d dFdv = (-dPdv).cross(interpolN) + (p - interpolP).cross(dNdv);

		jacobian.col(0) = dFdu;
		jacobian.col(1) = dFdv;

		//std::cout << uv.transpose() << " => " << F.transpose() << std::endl;

		Vector2d rhs = -jacobian.transpose() * F;
		auto lhs = jacobian.transpose() * jacobian;
		float norm = 1.0f / (lhs(0, 0) * lhs(1, 1) - lhs(0, 1) * lhs(1, 0));

		uv += Vector2d(lhs(1, 1) * rhs.x() - lhs(0, 1) * rhs.y(), -lhs(1, 0) * rhs.x() + lhs(0, 0) * rhs.y()) * norm;
	}

	interpolP = bilinear(quad_vs[0], quad_vs[1], quad_vs[2], quad_vs[3], uv);
	interpolN = bilinear(vs_normals[0], vs_normals[1], vs_normals[2], vs_normals[3], uv);
}
void projectPointOnTriangle(const vector<Vector3d>& tri_vs, const vector<Vector3d> & vs_normals, const Vector3d& p, Vector2d& uv, Vector3d& interpolP, Vector3d& interpolN)
{
	Eigen::Matrix<double, 3, 2> jacobian;

	uv = Vector2d::Constant(0.333f);

	//Newton iteration for projection on triangle

	//objective function:
	// F(u, v) = (p - interpolP) x interpolN = 0
	Vector3d F;

	Vector3d dPdu = tri_vs[0] - tri_vs[2];
	Vector3d dPdv = tri_vs[1] - tri_vs[2];
	Vector3d dNdu = vs_normals[0] - vs_normals[2];
	Vector3d dNdv = vs_normals[1] - vs_normals[2];

	for (int i = 0; i < 20; ++i)
	{
		interpolP = barycentric(tri_vs[0], tri_vs[1], tri_vs[2], uv);
		interpolN = barycentric(vs_normals[0], vs_normals[1], vs_normals[2], uv);

		F = (p - interpolP).cross(interpolN);
		Vector3d dFdu = (-dPdu).cross(interpolN) + (p - interpolP).cross(dNdu);
		Vector3d dFdv = (-dPdv).cross(interpolN) + (p - interpolP).cross(dNdv);

		jacobian.col(0) = dFdu;
		jacobian.col(1) = dFdv;

		//std::cout << uv.transpose() << " => " << F.transpose() << std::endl;

		Vector2d rhs = -jacobian.transpose() * F;
		auto lhs = jacobian.transpose() * jacobian;
		float norm = 1.0f / (lhs(0, 0) * lhs(1, 1) - lhs(0, 1) * lhs(1, 0));

		uv += Vector2d(lhs(1, 1) * rhs.x() - lhs(0, 1) * rhs.y(), -lhs(1, 0) * rhs.x() + lhs(0, 0) * rhs.y()) * norm;
	}

	interpolP = barycentric(tri_vs[0], tri_vs[1], tri_vs[2], uv);
	interpolN = barycentric(vs_normals[0], vs_normals[1], vs_normals[2], uv);
}
template <typename T>
T bilinear(const T& v1, const T& v2, const T& v3, const T& v4, const Vector2d& uv) {
	return (1 - uv.x()) * ((1 - uv.y()) * v1 + uv.y() * v4) + uv.x() * ((1 - uv.y()) * v2 + uv.y() * v3);
}
template <typename T>
T barycentric(const T& v1, const T& v2, const T& v3, const Vector2d& uv)
{
	return uv.x() * v1 + uv.y() * v2 + (1 - uv.x() - uv.y()) * v3;
}
template <typename T>
bool num_equal(const T& x, const T& y, const double &precision) {
	return std::abs(x - y) <= std::max(precision, precision * std::max(std::abs(x), std::abs(y)));
}
void MultiResolutionHierarchy::prolongPositions(int level) {
    const SMatrix &P = mP[level];

    for (int k = 0; k < P.outerSize(); ++k) {
        SMatrix::InnerIterator it(P, k);
        for (; it; ++it)
            mO[level].col(it.row()) = mO[level+1].col(it.col());
    }
}

void MultiResolutionHierarchy::detectPositionSingularitiesTri() {
    Timer<> timer;
    timer.beginStage("Computing position singularities");
    const MatrixXu &F = mF;
    const MatrixXf &V = mV[0], &O = mO[0], &N = mN[0], &NF = mNF, &Q = mQ[0];
    MatrixXf &S = mPositionSingularities;
    uint32_t singularityCount = 0;
    std::mutex mutex;

    tbb::parallel_for(
        tbb::blocked_range<uint32_t>(0u, (uint32_t) F.cols(), 1000),
        [&](const tbb::blocked_range<uint32_t> &range) {
            for (uint32_t f = range.begin(); f != range.end(); ++f) {
                uint32_t k[3] = { F(0, f), F(1, f), F(2, f) };
                Vector2i trans = Vector2i::Zero();
                Vector3f q_cur = Q.col(k[0]);
                Vector3f face_center = Vector3f::Zero();

                if (!combed()) {
                    for (int j = 0; j < 3; ++j) {
                        int n = (j + 1) % 3;
                        Vector3f q_next = applyRotationKeep(q_cur, N.col(k[j]), Q.col(k[n]), N.col(k[n]));
                        trans += findClosestPair(O.col(k[j]), q_cur,  N.col(k[j]), V.col(k[j]),
                                                 O.col(k[n]), q_next, N.col(k[n]), V.col(k[n]),
                                                 mScale, mInvScale).second;
                        q_cur = q_next;
                        face_center += V.col(k[j]);
                    }
                } else {
                    trans = mO_combed.col(3*f+0) + mO_combed.col(3*f+1) + mO_combed.col(3*f+2);
                    for (int j = 0; j < 3; ++j)
                        face_center += V.col(k[j]);
                }

                if (std::abs(q_cur.dot(Q.col(k[0])) - 1) > 1e-3)
                    continue;

                if (trans != Vector2i::Zero()) {
                    face_center *= 1.f / 3.f;
                    std::lock_guard<std::mutex> lock(mutex);

                    if (singularityCount + 1 > S.cols())
                        S.conservativeResize(9, S.cols() * 2 + 1);
                    S.col(singularityCount++)
                        << face_center + NF.col(f) * mAverageEdgeLength / 3, NF.col(f), Vector3f(1, 1, 0);
                }
            }
        }
    );
    S.conservativeResize(9, singularityCount);
    timer.endStage("Found " + std::to_string(singularityCount) + " singular faces");
}

void MultiResolutionHierarchy::detectPositionSingularitiesTet() {
    Timer<> timer;
    timer.beginStage("Computing position singularities");

	//projectBack3D();

    const MatrixXu &T = mT;
    const MatrixXf &V = mV[0], &O = mO[0], &Q = mQ[0];
    MatrixXf &S = mPositionSingularities;
    uint32_t singularityCount = 0;
    std::mutex mutex;
    uint8_t tet_faces[4][3] = { { 1, 0, 2 }, { 3, 2, 0 }, { 1, 2, 3 }, { 0, 1, 3 } };

    tbb::parallel_for(
        tbb::blocked_range<uint32_t>(0u, (uint32_t) T.cols(), 1000),
        [&](const tbb::blocked_range<uint32_t> &range) {
            for (uint32_t t = range.begin(); t != range.end(); ++t) {

                for (auto f_: tet_faces) {
                    uint32_t f[3] = { T(f_[0], t), T(f_[1], t), T(f_[2], t) };

                    Vector3i trans = Vector3i::Zero();
                    Quaternion q_cur = Q.col(f[0]);
                    for (int j = 0; j < 3; ++j) {
                        int n = (j + 1) % 3;
                        Quaternion q_next = Quaternion::applyRotation(Q.col(f[n]), q_cur);
                        trans += findClosestPair(O.col(f[j]), q_cur, O.col(f[n]),
                                             q_next, mScale, mInvScale).second;
                        q_cur = q_next;
                    }
                    if (std::abs(q_cur.dot(Q.col(f[0])) - 1) > 1e-3)
                        continue;

                    if (trans != Vector3i::Zero()) {
                        std::lock_guard<std::mutex> lock(mutex);

                        Vector3f tc =
                            0.25f * (V.col(T(0, t)) + V.col(T(1, t)) +
                                     V.col(T(2, t)) + V.col(T(3, t)));

                        Vector3f fc = (V.col(f[0]) + V.col(f[1]) +
                                       V.col(f[2])) * (1.f / 3.f);

                        if (singularityCount + 2 > S.cols())
                            S.conservativeResize(6, 2*S.cols() + 2);
                        Vector3f color(1,1,0);
                        S.col(singularityCount++) << tc, color;
                        S.col(singularityCount++) << fc, color;
                    }
                }
            }
        }
    );
    S.conservativeResize(6, singularityCount);
    timer.endStage("Found " + std::to_string(singularityCount) + " singular faces");

	char path[1024], path_[1024];
	strcpy(path_, outpath.c_str());
	strncpy(path_, outpath.c_str(), sizeof(path_));
	path_[sizeof(path_) - 1] = 0;
	sprintf(path, "%s%s", path_, "_Posy.sing");
	write_singularities_SING(S, path);
}
