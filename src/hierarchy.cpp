#include "hierarchy.h"
#include "meshio.h"
#include "timer.h"
#include "quat.h"
#include "bvh.h"
#include "orient_triangle_mesh.h"
#include "dedge.h"
#include "subdivide.h"
#include "tri_tri_intersection.h"

MultiResolutionHierarchy::MultiResolutionHierarchy() {
    mV = { MatrixXf::Zero(3, 1) };
    mN = { MatrixXf::Zero(3, 1) };
    mO = { MatrixXf::Zero(3, 1) };
    mQ = { MatrixXf::Zero(4, 1) };
    mBVH = nullptr;
	ratio_scale = 3.0;
	tElen_ratio = 1.0;
	tet_elen = 0.6;
	re_color = true;
	Qquadric = splitting = decomposes = doublets = triangles = false;
	global_parameterization = false;
}

bool MultiResolutionHierarchy::load(const std::string &filename) {
    std::lock_guard<ordered_lock> lock(mMutex);

    mV.resize(1);
	mV[0] = MatrixXf::Zero(3, 1);
	mF = MatrixXu::Zero(3, 1);
    try {
		//loadTetMesh(filename, mV[0], mF, mT);
		loadTetMesh_VTK(filename, mV[0], mF, mT);
    } catch (const std::exception &e) {
        std::cout << "failed loading vtk file." << std::endl;

		try {
			Mesh mesh; Mesh_Quality mq;
			load_HYBRID_mesh(mesh, filename);
			convert_to_tri_hybridmesh(mesh);
			compute_volume_ratio(mesh, mq);
			//compute_self_intersection(mesh, mq);

			return true;
		}
		catch (const std::exception &e) {
			std::cout << "failed loading HYRBID file." << std::endl;
			return false;
		}

        try {
			load_obj(filename, mF, mV[0]);
        } catch (const std::exception &e) {
            std::cout << "failed loading obj file." << std::endl;
			return false;
        }
		try {
			std::string filename_para = filename + ".2Dpara";
			load_obj(filename_para, PF, PV);
			//compute the vertex id maps
			V2PV_map.clear(); V2PV_map.resize(mV[0].cols());
			PV2V_map.clear(); PV2V_map.resize(PV.cols()); fill(PV2V_map.begin(), PV2V_map.end(), -1);
			for (uint32_t i = 0; i < mF.cols(); i++) {
				for (uint32_t j = 0; j < 3; j++){
					if(find(V2PV_map[mF(j, i)].begin(), V2PV_map[mF(j, i)].end(), PF(j, i)) == V2PV_map[mF(j, i)].end())
						V2PV_map[mF(j, i)].push_back(PF(j, i));
					PV2V_map[PF(j, i)] = mF(j, i);
				}
			}
			global_parameterization = true;
		}catch (const std::exception &e) {
			std::cout << "no global para." << std::endl;
		}
    }
	if (tetMesh()) {
		//Fs
		std::vector<std::tuple<uint32_t, uint32_t, uint32_t, uint32_t, bool>> tempF;
		tempF.reserve(mT.cols() * 4);
		std::vector<Vector3u> Fs;
		for (uint32_t t = 0; t < mT.cols(); ++t) {
			for (uint32_t f = 0; f < 4; ++f) {
				uint32_t v0 = mT(tet_faces[f][0], t), v1 = mT(tet_faces[f][1], t), v2 = mT(tet_faces[f][2], t);
				if (v0 > v1) std::swap(v0, v1);
				if (v1 > v2) std::swap(v2, v1);
				if (v0 > v1) std::swap(v0, v1);
				tempF.push_back(std::make_tuple(v0, v1, v2, t, true));
			}
		}
		std::sort(tempF.begin(), tempF.end());
		Fs.clear();
		Fs.reserve(tempF.size() / 3);
		int F_num = -1, f_b=0; std::vector<bool> f_boundary; f_boundary.reserve(tempF.size() / 2);
		for (uint32_t i = 0; i < tempF.size(); ++i) {
			if (i == 0 || (i != 0 &&
				(std::get<0>(tempF[i]) != std::get<0>(tempF[i - 1]) ||
					std::get<1>(tempF[i]) != std::get<1>(tempF[i - 1]) ||
					std::get<2>(tempF[i]) != std::get<2>(tempF[i - 1])))) {
				F_num++;
				Vector3u v(std::get<0>(tempF[i]), std::get<1>(tempF[i]), std::get<2>(tempF[i]));
				Fs.push_back(v);
				f_boundary.push_back(true);
				f_b++;
			}
			else {
				f_boundary[F_num] = false;
				f_b--;
			}
		}
		mF.resize(3, f_b); f_b = 0;
		for (uint32_t f = 0; f < F_num; f++)
			if(f_boundary[f]) mF.col(f_b++) = Fs[f];

		orient_triangle_mesh_index(mV[0], mF);

		std::cout << "V, F, T: " << mV[0].cols() << " " << mF.cols() << " " << mT.cols() << endl;
	}
	else {
		//orient_triangle_mesh_index(mV[0], mF);
		std::cout << "V, F: " << mV[0].cols() << " " << mF.cols() << endl;
		//char obj_mesh[300];
		//sprintf(obj_mesh, "%s%s", "C:/Users/XifengGao/Desktop/Osurface", ".obj");
		//write_surface_mesh_OBJ(mV[0], mF, obj_mesh);
	}

	mV.resize(1);
	mAABB = AABB(
		mV[0].rowwise().minCoeff(),
		mV[0].rowwise().maxCoeff()
	);

	ms = compute_mesh_stats(mF, mV[0]);
	tet_elen = tElen_ratio * ms.mAverageEdgeLength;


    //build();
    return true;
}
MeshStats MultiResolutionHierarchy::compute_mesh_stats(const MatrixXu &F_, const MatrixXf &V_, bool deterministic)
{
	MeshStats stats;
	cout << "Computing mesh statistics .. ";
	cout.flush();
	auto map = [&](const tbb::blocked_range<uint32_t> &range, MeshStats stats) -> MeshStats {
		for (uint32_t f = range.begin(); f != range.end(); ++f) {
			Vector3f v[3] = { V_.col(F_(0, f)), V_.col(F_(1, f)), V_.col(F_(2, f)) };
			Vector3f face_center = Vector3f::Zero();

			for (int i = 0; i<3; ++i) {
				Float edge_length = (v[i] - v[i == 2 ? 0 : (i + 1)]).norm();
				stats.mAverageEdgeLength += edge_length;
				stats.mMaximumEdgeLength = std::max(stats.mMaximumEdgeLength, (double)edge_length);
				stats.mAABB.expandBy(v[i]);
				face_center += v[i];
			}
			face_center *= 1.0f / 3.0f;

			Float face_area = 0.5f * (v[1] - v[0]).cross(v[2] - v[0]).norm();
			stats.mSurfaceArea += face_area;
			stats.mWeightedCenter += face_area * face_center;
		}
		return stats;
	};

	auto reduce = [](MeshStats s0, MeshStats s1) -> MeshStats {
		MeshStats result;
		result.mSurfaceArea = s0.mSurfaceArea + s1.mSurfaceArea;
		result.mWeightedCenter = s0.mWeightedCenter + s1.mWeightedCenter;
		result.mAverageEdgeLength =
			s0.mAverageEdgeLength + s1.mAverageEdgeLength;
		result.mMaximumEdgeLength =
			std::max(s0.mMaximumEdgeLength, s1.mMaximumEdgeLength);
		result.mAABB = AABB::merge(s0.mAABB, s1.mAABB);
		return result;
	};

	tbb::blocked_range<uint32_t> range(0u, (uint32_t)F_.cols(), GRAIN_SIZE);

	if (deterministic)
		stats = tbb::parallel_deterministic_reduce(range, MeshStats(), map, reduce);
	else
		stats = tbb::parallel_reduce(range, MeshStats(), map, reduce);

	stats.mAverageEdgeLength /= F_.cols() * 3;
	stats.mWeightedCenter /= stats.mSurfaceArea;

	return stats;
}
bool MultiResolutionHierarchy::tet_meshing()
{
	MatrixXf V;
	MatrixXu F;
	if(!tetMesh()){
		V = mV[0];
		F = mF;
	}
	else {
		V = mV[0];
		F = mF;
		//return false;
	}
	Vector3f minV = mV[0].rowwise().minCoeff() + Vector3f(-0.1, -0.1, -0.1),
		maxV = mV[0].rowwise().maxCoeff() + Vector3f(0.1, 0.1, 0.1);
	//std::vector<std::vector<Float>> Vs(8);
	MatrixXf Vs(3, 8);
	for(uint32_t i=0;i<8;i++)
		for (uint32_t j = 0; j < 3; j++) {
			short bit = ((1 << j) & i) >> j;
			//Vs[i].push_back(bit *minV[j] + (1 - bit)*maxV[j]);
			Vs(j, i) = (bit *minV[j] + (1 - bit)*maxV[j]);
		}
	MatrixXu tris_Cube(12, 3);
	tris_Cube <<
		0, 2, 3,
		0, 3, 1,
		3, 2, 7,
		7, 2, 6,
		3, 7, 5,
		3, 5, 1,
		1, 5, 0,
		0, 5, 4,
		4, 5, 7,
		4, 7, 6,
		4, 6, 2,
		4, 2, 0;
	tris_Cube.transposeInPlace();
	orient_triangle_mesh_index(Vs, tris_Cube);

	//MatrixXu tets_Cube(4, 6);
	//tets_Cube << 2, 4, 7, 6,
	//	2, 4, 3, 0,
	//	4, 2, 7, 3,
	//	0, 3, 4, 1,
	//	4, 3, 1, 5,
	//	3, 4, 5, 7;
	tetgenio in_bg_, in, addin, in_bg, out_, out;

	in_bg_.numberofpoints = 8;
	in_bg_.pointlist = new REAL[8 * 3];
	for (uint32_t i = 0; i < 8; i++)
		for (uint32_t j = 0; j < 3; j++)
			//in_bg_.pointlist[3* i+j] = Vs[i][j];
			in_bg_.pointlist[3 * i + j] = Vs(j, i);
	in_bg_.numberoffacets = 12;
	in_bg_.facetlist = new tetgenio::facet[12];
	in_bg_.facetmarkerlist = new int[in_bg_.numberoffacets];
	tetgenio::facet *f0;
	tetgenio::polygon *p0;
	for (uint32_t i = 0; i < in_bg_.numberoffacets; i++) {
		// Facet 1. The leftmost facet.
		f0 = &in_bg_.facetlist[i];
		f0->numberofpolygons = 1;
		f0->polygonlist = new tetgenio::polygon[f0->numberofpolygons];
		f0->numberofholes = 0;
		f0->holelist = NULL;
		p0 = &f0->polygonlist[0];
		p0->numberofvertices = 3;
		p0->vertexlist = new int[p0->numberofvertices];
		p0->vertexlist[0] = tris_Cube(0, i);
		p0->vertexlist[1] = tris_Cube(1, i);
		p0->vertexlist[2] = tris_Cube(2, i);
		// Set 'in.facetmarkerlist'
		in_bg_.facetmarkerlist[i] = 1;
	}
	//in_bg_.numberoftetrahedra = 6;
	//in_bg_.tetrahedronlist = new int[6 * 4];
	//for (uint32_t i = 0; i < 6; i++)
	//	for (uint32_t j = 0; j < 4; j++)
	//	in_bg_.tetrahedronlist[4* i + j] = tets_Cube(j, i);
	tetrahedralize("pq", &in_bg_, &out_);

	in_bg.numberofpoints = out_.numberofpoints;
	in_bg.pointlist = new REAL[in_bg.numberofpoints * 3];
	for (uint32_t i = 0; i < 3 * out_.numberofpoints; i++)
		in_bg.pointlist[i] = out_.pointlist[i];
	in_bg.numberoftetrahedra = out_.numberoftetrahedra;
	in_bg.tetrahedronlist = new int[out_.numberoftetrahedra * 4];
	for (uint32_t i = 0; i < 4 * out_.numberoftetrahedra; i++)
		in_bg.tetrahedronlist[i] = out_.tetrahedronlist[i];
	in_bg.pointmtrlist = new double[in_bg.numberofpoints];
	in_bg.numberofpointmtrs = 1;
	std::cout << "target tet edge len: " << tet_elen << endl;
	for (int i = 0; i<in_bg.numberofpoints; i++)
		in_bg.pointmtrlist[i] = tet_elen;
	

	tetgenio::facet *f;
	tetgenio::polygon *p;

	// All indices start from 1.
	in.firstnumber = 0;

	in.numberofpoints = V.cols();
	in.pointlist = new REAL[in.numberofpoints * 3];
	in.pointmarkerlist = new int[in.numberofpoints];
	in.pointmtrlist = new double[in.numberofpoints];
	for (int i = 0; i<in.numberofpoints; i++)
	{
		in.pointlist[3 * i + 0] = V(0,i);
		in.pointlist[3 * i + 1] = V(1,i);
		in.pointlist[3 * i + 2] = V(2,i);
		in.pointmarkerlist[i] = 1;
	}

	in.numberoffacets = F.cols();
	in.facetlist = new tetgenio::facet[in.numberoffacets];
	in.facetmarkerlist = new int[in.numberoffacets];

	for (int i = 0; i<in.numberoffacets; i++)
	{
		// Facet 1. The leftmost facet.
		f = &in.facetlist[i];
		f->numberofpolygons = 1;
		f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
		f->numberofholes = 0;
		f->holelist = NULL;
		p = &f->polygonlist[0];
		p->numberofvertices = 3;
		p->vertexlist = new int[p->numberofvertices];
		p->vertexlist[0] = F(0,i);
		p->vertexlist[1] = F(1,i);
		p->vertexlist[2] = F(2,i);

		// Set 'in.facetmarkerlist'
		in.facetmarkerlist[i] = 1;
	}
	tetrahedralize("pqm", &in, &out, &addin, &in_bg);//pq1.414Va0.0001;q1.1
	//tetrahedralize("pq", &in, &out_);//pq1.414Va0.0001;q1.1
	//in_bg.numberofpoints = out_.numberofpoints;
	//in_bg.pointlist = new REAL[in_bg.numberofpoints * 3];
	//for (uint32_t i = 0; i < 3 * out_.numberofpoints; i++)
	//	in_bg.pointlist[i] = out_.pointlist[i];
	//in_bg.numberoftetrahedra = out_.numberoftetrahedra;
	//in_bg.tetrahedronlist = new int[out_.numberoftetrahedra * 4];
	//for (uint32_t i = 0; i < 4 * out_.numberoftetrahedra; i++)
	//	in_bg.tetrahedronlist[i] = out_.tetrahedronlist[i];
	//in_bg.pointmtrlist = new double[in_bg.numberofpoints];
	//in_bg.numberofpointmtrs = 1;
	//std::cout << "target tet edge len: " << tet_elen << endl;
	//for (int i = 0; i<in_bg.numberofpoints; i++)
	//{
	//	in_bg.pointmtrlist[i] = tet_elen;
	//}
	//
	//tetrahedralize("pqm", &in, &out, &addin, &in_bg);//pq1.414Va0.0001;q1.1
	
	////tetrahedralize(reinterpret_cast<tetgenbehavior *>("pq1.414Y"), &in, &out);//pq1.414Va0.0001;q1.1

	mV[0].setZero(); mV[0].resize(3, out.numberofpoints);
	for (uint32_t i = 0; i < out.numberofpoints; i++) {
		for (uint32_t j = 0; j < 3; j++)
			mV[0](j, i) = out.pointlist[3 * i + j];
	}
	mT.setZero(); mT.resize(4, out.numberoftetrahedra);
	for (int i = 0; i < out.numberoftetrahedra; i++){
		for (uint32_t j = 0; j < 4; j++)
			mT(j, i) = out.tetrahedronlist[4 * i + j];
	}

	//Fs
	std::vector<std::tuple<uint32_t, uint32_t, uint32_t, uint32_t, bool>> tempF;
	tempF.reserve(mT.cols() * 4);
	std::vector<Vector3u> Fs;
	for (uint32_t t = 0; t < mT.cols(); ++t) {
		for (uint32_t f = 0; f < 4; ++f) {
			uint32_t v0 = mT(tet_faces[f][0], t), v1 = mT(tet_faces[f][1], t), v2 = mT(tet_faces[f][2], t);
			if (v0 > v1) std::swap(v0, v1);
			if (v1 > v2) std::swap(v2, v1);
			if (v0 > v1) std::swap(v0, v1);
			tempF.push_back(std::make_tuple(v0, v1, v2, t, true));
		}
	}
	std::sort(tempF.begin(), tempF.end());
	Fs.clear();
	Fs.reserve(tempF.size() / 3);
	int F_num = -1, f_b = 0; std::vector<bool> f_boundary; f_boundary.reserve(tempF.size() / 2);
	for (uint32_t i = 0; i < tempF.size(); ++i) {
		if (i == 0 || (i != 0 &&
			(std::get<0>(tempF[i]) != std::get<0>(tempF[i - 1]) ||
				std::get<1>(tempF[i]) != std::get<1>(tempF[i - 1]) ||
				std::get<2>(tempF[i]) != std::get<2>(tempF[i - 1])))) {
			F_num++;
			Vector3u v(std::get<0>(tempF[i]), std::get<1>(tempF[i]), std::get<2>(tempF[i]));
			Fs.push_back(v);
			f_boundary.push_back(true);
			f_b++;
		}
		else {
			f_boundary[F_num] = false;
			f_b--;
		}
	}
	mF.resize(3, f_b); f_b = 0;
	for (uint32_t f = 0; f < F_num; f++)
		if (f_boundary[f]) mF.col(f_b++) = Fs[f];

	orient_triangle_mesh_index(mV[0], mF);

	std::cout << "V, F, T: " << mV[0].cols() << " " << mF.cols() << " " << mT.cols() << endl;
	return true;
}
void MultiResolutionHierarchy::build() {

    Timer<> timer;
    mV.resize(1);

    if (mBVH)
        delete mBVH;

    timer.beginStage("Computing face and vertex normals");
    mN.resize(1);
    mC.resize(1);
	nV_boundary_flag.resize(1);
    mN[0].setZero(3, mV[0].cols());
    mC[0].setZero(3, mV[0].cols());
    mNF.resize(3, mF.cols()); mCF.resize(3, mF.cols());
    VectorXi count(mV[0].cols());
    count.setZero();
    for (uint32_t i=0; i<mF.cols(); ++i) {
        uint32_t i0 = mF(0, i), i1 = mF(1, i), i2 = mF(2, i);
        Vector3f v0 = mV[0].col(i0), v1 = mV[0].col(i1), v2 = mV[0].col(i2);
        Vector3f n = (v1 - v0).cross(v2 - v0).normalized();
        mNF.col(i) = n;
		mCF.col(i) += (v0 + v1 + v2) / 3;
        mN[0].col(i0) += n; mN[0].col(i1) += n; mN[0].col(i2) += n;
        count[i0]++; count[i1]++; count[i2]++;
    }

    for (uint32_t i=0; i<mN[0].cols(); ++i) {
        if (mN[0].col(i) != Vector3f::Zero()) {
            Vector3f d1 = mN[0].col(i) / count[i],
                     d2 = mN[0].col(i).normalized();
			//if (tetMesh() && d1.dot(d2) < 0.99f)
			if (tetMesh() && d1.dot(d2) < 0.85f)
				d2 = Vector3f::Zero();;//
            mN[0].col(i) = d2;
            if (d2 != Vector3f::Zero())
                mC[0].col(i) = mV[0].col(i);
        }
    }

	vnfs.clear();
	vnfs.resize(mV[0].cols());
	for (uint32_t i = 0; i < mF.cols(); ++i) for (uint32_t j = 0; j < 3; j++) vnfs[mF(j, i)].push_back(i);

	
	timer.endStage();

    timer.beginStage("Computing adjacency data structure");
	mL.clear(); mP.clear();

    if (tetMesh()) { 
        std::vector<std::pair<uint32_t, uint32_t>> adj;
        adj.reserve(mT.cols() * 12);
        for (uint32_t t = 0; t < mT.cols(); ++t) {
            const int tet_edges[6][2] = {
                { 0, 1 }, { 0, 2 }, { 0, 3 }, { 1, 2 }, { 1, 3 }, { 2, 3 } };
            for (int i = 0; i < 6; ++i) {
                uint32_t v0 = mT(tet_edges[i][0], t);
                uint32_t v1 = mT(tet_edges[i][1], t);
                adj.push_back(std::make_pair(v0, v1));
                adj.push_back(std::make_pair(v1, v0));
            }
        }
        std::sort(adj.begin(), adj.end());
        adj.erase(std::unique(adj.begin(), adj.end()), adj.end());

        std::vector<Triplet> triplets;
        for (auto item : adj)
            triplets.push_back(Triplet(item.first, item.second, 1.f));
        mL.resize(1);
        mL[0].resize(mV[0].cols(), mV[0].cols());
        mL[0].setFromTriplets(triplets.begin(), triplets.end());
    } else {

		construct_tEs_tFEs(mF, nFes, nEs);
		//nV_nes, tag boundary V
		nV_boundary_flag[0].clear(); nV_boundary_flag[0].resize(mV[0].cols(), false);
		nV_nes.clear(); nV_nes.resize(mV[0].cols());
		for (uint32_t i = 0; i < nEs.size(); i++) { 
			uint32_t v0 = std::get<0>(nEs[i]);
			uint32_t v1 = std::get<1>(nEs[i]);
			nV_nes[v0].push_back(i);
			nV_nes[v1].push_back(i);
			if (std::get<2>(nEs[i])) {
				nV_boundary_flag[0][v0] = nV_boundary_flag[0][v1] = true;
			}
		}

        std::vector<std::pair<uint32_t, uint32_t>> adj;
        adj.reserve(mF.cols() * 6);
        for (uint32_t f = 0; f < mF.cols(); ++f) {
            for (int i = 0; i < 3; ++i) {
                uint32_t v0 = mF(i, f);
                uint32_t v1 = mF((i + 1) % 3, f);
                adj.push_back(std::make_pair(v0, v1));
                adj.push_back(std::make_pair(v1, v0));
            }
        }
        std::sort(adj.begin(), adj.end());
        adj.erase(std::unique(adj.begin(), adj.end()), adj.end());

        std::vector<Triplet> triplets;
        for (auto item : adj)
            triplets.push_back(Triplet(item.first, item.second, 1.f));
        mL.resize(1);
        mL[0].resize(mV[0].cols(), mV[0].cols());
        mL[0].setFromTriplets(triplets.begin(), triplets.end());
    }
    for (uint32_t i = 0; i < (uint32_t) mL[0].rows(); ++i) {
        Float sum = 1 / mL[0].row(i).sum();
        mL[0].row(i) *= sum;
        mL[0].coeffRef(i, i) = -sum;
    }
    mL[0].makeCompressed();
    timer.endStage();

    struct WeightedEdge {
        WeightedEdge(uint32_t _i0, uint32_t _i1, Float weight)
            : weight(weight), i0(_i0), i1(_i1) {
            if (i0 > i1)
                std::swap(i0, i1);
        }

        bool operator<(const WeightedEdge &e) const {
            return std::tie(weight, i0, i1) < std::tie(e.weight, e.i0, e.i1);
        }

        Float weight;
        uint32_t i0, i1;
    };

    timer.beginStage("Building hierarchy");
    while (mL[mL.size() - 1].cols() > 1) {
        const MatrixXf &V = mV[mV.size() - 1];
        const MatrixXf &N = mN[mN.size() - 1];
        const MatrixXf &C = mC[mC.size() - 1];
		const vector<bool> &VB = nV_boundary_flag[nV_boundary_flag.size() - 1];
        const SMatrix &L = mL[mL.size() - 1];
        std::vector<bool> collapsed(L.cols(), false);
        std::vector<bool> visited(L.cols(), false);
        std::set<WeightedEdge> edges;

        double edgeSum = 0;
        size_t edgeCount = 0;
        for (int k = 0; k < L.outerSize(); ++k) {
            for (SMatrix::InnerIterator it(L, k); it; ++it) {
                if (it.col() == it.row())
                    continue;
                Float length = (V.col(it.row()) - V.col(it.col())).norm();
                edgeSum += length;
                edgeCount += 1;
                edges.insert(WeightedEdge(it.row(), it.col(), length));
            }
        }
        if (mL.size() == 1)
            mAverageEdgeLength = edgeSum / edgeCount;

        std::vector<Triplet> P_triplets, R_triplets;
        std::vector<Vector3f> V_next, N_next, C_next;
        std::map<uint32_t, uint32_t> vertex_map;

		uint32_t nVertices = 0; vector<bool> vb_flag(V.cols(), false);
        for (auto const &e : edges) {
            visited[e.i0] = visited[e.i1] = true;
            if (collapsed[e.i0] || collapsed[e.i1])
                continue;
            collapsed[e.i0] = true;
            collapsed[e.i1] = true;
            P_triplets.push_back(Triplet(e.i0, nVertices, 1.0f));
            P_triplets.push_back(Triplet(e.i1, nVertices, 1.0f));
            R_triplets.push_back(Triplet(nVertices, e.i0, 0.5f));
            R_triplets.push_back(Triplet(nVertices, e.i1, 0.5f));
            V_next.push_back(0.5f * (V.col(e.i0) + V.col(e.i1)));

			if (VB[e.i0] || VB[e.i1]) vb_flag[nVertices] = true;

            Vector3f n = N.col(e.i0) + N.col(e.i1);
            Vector3f c = C.col(e.i0) + C.col(e.i1);
            if (N.col(e.i0) != Vector3f::Zero() && 
                N.col(e.i1) != Vector3f::Zero()) {
                if (tetMesh()) {
                    n = N.col(e.i0);
                    c = C.col(e.i0);
                } else {
                    n.normalize();
                    c *= 0.5f;
                }
            }

            N_next.push_back(n);
            C_next.push_back(c);

            vertex_map[e.i0] = nVertices;
            vertex_map[e.i1] = nVertices;
            nVertices++;
        }

        for (uint32_t i=0; i<V.cols(); ++i) {
            if (collapsed[i] || !visited[i])
                continue;
            P_triplets.push_back(Triplet(i, nVertices, 1.0f));
            R_triplets.push_back(Triplet(nVertices, i, 1.0f));
            V_next.push_back(V.col(i));
            N_next.push_back(N.col(i));
            C_next.push_back(C.col(i));
            vertex_map[i] = nVertices;

			if (VB[i]) vb_flag[nVertices] = true;

            nVertices++;
        }
		vb_flag.resize(nVertices);

        if (mL.size() != 1)
            std::cout << ", ";
        std::cout << nVertices;
        std::cout.flush();

        SMatrix P(V.cols(), nVertices), R(nVertices, V.cols());

        P.setFromTriplets(P_triplets.begin(), P_triplets.end());
        R.setFromTriplets(R_triplets.begin(), R_triplets.end());

        SMatrix L2 = R*L*P;
        MatrixXf V2(3, nVertices), N2(3, nVertices), C2(3, nVertices), Q2(4, nVertices);
        for (uint32_t i=0; i<nVertices; ++i) {
            V2.col(i) = V_next[i];
            N2.col(i) = N_next[i];
            C2.col(i) = C_next[i];
        }

		nV_boundary_flag.push_back(vb_flag);
        mP.push_back(std::move(P));
        mN.push_back(std::move(N2));
        mV.push_back(std::move(V2));
        mC.push_back(std::move(C2));
        mL.push_back(L2);
    }
    std::cout << " ";
    timer.endStage();

    mQ.resize(mL.size());
    mO.resize(mL.size());

    pcg32 rng;
    if (tetMesh()) {
        for (uint32_t i = 0; i < mL.size(); ++i) {
            mQ[i].resize(4, mV[i].cols());
            mO[i].resize(3, mV[i].cols());
            for (uint32_t j = 0; j < mV[i].cols(); ++j) {
                mQ[i].col(j) = Quaternion::Random(rng);

                mO[i].col(j) = aabbRand(mAABB, rng);
            }
        }

		//Vector4f direction = Quaternion::Random(rng);

		//for (uint32_t i = 0; i < mL.size(); ++i) {
		//	mQ[i].resize(4, mV[i].cols());
		//	mO[i].resize(3, mV[i].cols());
		//	for (uint32_t j = 0; j < mV[i].cols(); ++j) {
		//		//mQ[i].col(j) = Quaternion::Random(rng);
		//		mQ[i].col(j) = direction;
		//		mO[i].col(j) = aabbRand(mAABB, rng);
		//	}
		//}
    } else {
        for (uint32_t i = 0; i < mL.size(); ++i) {
            mQ[i].resize(3, mV[i].cols());
            mO[i].resize(3, mV[i].cols());

			for (uint32_t j = 0; j < mV[i].cols(); ++j) {
				if (i == 0 && nV_boundary_flag[i][j]) {
					std::vector<uint32_t> vs;
					for (auto eid : nV_nes[j]) {
						if (!std::get<2>(nEs[eid])) continue;

						uint32_t v0 = std::get<0>(nEs[eid]);
						uint32_t v1 = std::get<1>(nEs[eid]);
						if (v0 == j) vs.push_back(v1);
						else vs.push_back(v0);
					}
					Vector3f direct0, direct1; direct1.setZero();
					direct0 = (mV[0].col(j) - mV[0].col(vs[0])).normalized();
					for (uint32_t k = 1; k < vs.size(); k++) {
						Vector3f direct_ = (mV[0].col(vs[k]) - mV[0].col(j)).normalized();
						direct1 += direct_;
					}

					if (std::abs(direct0.dot(direct1)) < 0.5)
						mQ[i].col(j) = direct0;
					else
						mQ[i].col(j) = (direct0 + direct1).normalized();
					continue;
				}

				Vector3f n = mN[i].col(j), v = mV[i].col(j);
				Vector3f s, t;
				coordinate_system(n, s, t);
				float angle = rng.nextFloat() * 2 * M_PI;
				mQ[i].col(j) = s * std::cos(angle) + t * std::sin(angle);
			}


            for (uint32_t j = 0; j < mV[i].cols(); ++j) {
                Vector3f n = mN[i].col(j), v = mV[i].col(j);
				rng.nextFloat();
                Vector3f o = aabbRand(mAABB, rng);
                o -= n.dot(o - v) * n;
                mO[i].col(j) = o;
            }
        }
		//propagate up
		for (uint32_t i = 1; i < mL.size(); ++i) {
			for (int k = 0; k < mP[i - 1].outerSize(); ++k) {
				SMatrix::InnerIterator it(mP[i - 1], k);
				for (; it; ++it) {
					if (nV_boundary_flag[i - 1][it.row()]) 
						mQ[i].col(it.col()) = mQ[i - 1].col(it.row());
				}
			}
		}
    }
    mOrientationIterations = 0;
    mPositionIterations = 0;

    mBVH = new BVH(&mF, &mV[0], mAABB);
    mBVH->build();
	mScale = ms.mMaximumEdgeLength * ratio_scale;
	mInvScale = 1.f / mScale;

	sta.tN = mF.cols();
	sta.tetN = mT.cols();

}
void MultiResolutionHierarchy::construct_tEs_tFEs(MatrixXu & F, std::vector<std::vector<uint32_t>> &mtFes, std::vector<tuple_E> &mtEs) {
	mtFes.clear(); mtEs.clear();

	std::vector<std::tuple<uint32_t, uint32_t, uint32_t, uint32_t, int>> temp;
	temp.reserve(F.cols() * 3);
	mtFes.resize(F.cols());
	for (uint32_t f = 0; f < F.cols(); ++f) {
		for (uint32_t e = 0; e < 3; ++e) {
			uint32_t v0 = F(e, f), v1 = F((e + 1) % 3, f);
			if (v0 > v1) std::swap(v0, v1);
			temp.push_back(std::make_tuple(v0, v1, f, e, Edge_tag::B));
		}
		std::vector<uint32_t> fes(3);
		mtFes[f] = fes;
	}
	std::sort(temp.begin(), temp.end());
	mtEs.reserve(temp.size() / 2);
	int E_num = -1;
	for (uint32_t i = 0; i < temp.size(); ++i) {
		if (i == 0 || (i != 0 && (std::get<0>(temp[i]) != std::get<0>(temp[i - 1]) || std::get<1>(temp[i]) != std::get<1>(temp[i - 1])))) {
			E_num++;
			mtEs.push_back(std::make_tuple(std::get<0>(temp[i]), std::get<1>(temp[i]), true, 0, std::get<4>(temp[i]), E_num, -1, 0));
		}
		else if (i != 0 && (std::get<0>(temp[i]) == std::get<0>(temp[i - 1]) &&
			std::get<1>(temp[i]) == std::get<1>(temp[i - 1])))
			std::get<2>(mtEs[E_num]) = false;

		mtFes[std::get<2>(temp[i])][std::get<3>(temp[i])] = E_num;
	}
}

void MultiResolutionHierarchy::save(Serializer &serializer) const {
}

void MultiResolutionHierarchy::load(Serializer &serializer) {
}

//========HYBRID===========//
void MultiResolutionHierarchy::convert_to_tri_hybridmesh(Mesh &hmi) {

	Mesh mesh;
	mesh.V.resize(3, hmi.V.cols() + hmi.Fs.size()); mesh.V.setZero();
	mesh.V.block(0, 0, 3, hmi.V.cols()) = hmi.V;

	mesh.Fs.clear();
	vector<vector<int32_t>> F_map(hmi.Fs.size());
	for (auto f : hmi.Fs) {
		int f_size = f.vs.size();
		Vector3d center;
		center.setZero();
		for (int j = 0; j < f_size; j++) center += hmi.V.col(f.vs[j]);
		center /= f_size;
		mesh.V.col(hmi.V.cols() + f.id) = center;

		for (int j = 0; j < f_size; j++){
			Hybrid_F f_;
			f_.id = mesh.Fs.size();
			f_.vs.push_back(hmi.V.cols() + f.id);
			f_.vs.push_back(f.vs[j]);
			f_.vs.push_back(f.vs[(j + 1) % f_size]);
			mesh.Fs.push_back(f_);

			F_map[f.id].push_back(f_.id);
		}
	}
	int32_t onV = hmi.V.cols();
	hmi.V = mesh.V;
	hmi.Fs = mesh.Fs;
	for (auto &h : hmi.Hs) {
		Hybrid h_ = h; 
		for (auto f : h_.fs) h_.vs.push_back(onV + f);
		h_.fs.clear();
		for (uint32_t j = 0; j < h.fs.size(); j++) h_.fs.insert(h_.fs.end(), F_map[h.fs[j]].begin(), F_map[h.fs[j]].end());
		h = h_;
	}
}
void MultiResolutionHierarchy::compute_volume_ratio(Mesh &hmi, Mesh_Quality &mq){

	mq.H_Vols.resize(hmi.Hs.size()); mq.H_Vols.setZero();
	double hV = 0, pV = 0;
	for (auto h : hmi.Hs) {
		Vector3d center;
		center.setZero();
		for (auto v:h.vs) center += hmi.V.col(v);
		center /= h.vs.size();

		double hv = 0;
		for (auto f:h.fs) {
			Vector3d x = hmi.V.col(hmi.Fs[f].vs[0]) - center, y = hmi.V.col(hmi.Fs[f].vs[1]) - center, z = hmi.V.col(hmi.Fs[f].vs[2]) - center;
			hv += std::abs(-((x[0] * y[1] * z[2] + x[1] * y[2] * z[0] + x[2] * y[0] * z[1]) - (x[2] * y[1] * z[0] + x[1] * y[0] * z[2] + x[0] * y[2] * z[1])));
		}
		pV += hv;
		if (h.hex) hV += hv;
		mq.H_Vols[h.id] = hv;
	}
	cout << hV / pV << endl;
}
void MultiResolutionHierarchy::compute_self_intersection(Mesh &hmi, Mesh_Quality &mq) {

	mq.Self_In.resize(hmi.Hs.size()); mq.Self_In.setZero();
	int32_t inN = 0;
	for (auto h : hmi.Hs) {
		bool intersect = false;
		for (uint32_t i = 0; i < h.fs.size(); i++) {
			for (uint32_t j = i + 1; j < h.fs.size(); j++) {
				bool sharev = false;
				for (auto v0 : hmi.Fs[h.fs[i]].vs)for (auto v1 : hmi.Fs[h.fs[j]].vs) if (v0 == v1) { sharev = true; break; }
				if (sharev) continue;
				if (!sharev) {
					double V0[3], V1[3], V2[3], U0[3], U1[3], U2[3];
					for (uint32_t k = 0; k < 3; k++) {
						V0[k] = hmi.V(k, hmi.Fs[h.fs[i]].vs[0])*1000;
						V1[k] = hmi.V(k, hmi.Fs[h.fs[i]].vs[1])*1000;
						V2[k] = hmi.V(k, hmi.Fs[h.fs[i]].vs[2])*1000;

						U0[k] = hmi.V(k, hmi.Fs[h.fs[j]].vs[0]) * 1000;
						U1[k] = hmi.V(k, hmi.Fs[h.fs[j]].vs[1]) * 1000;
						U2[k] = hmi.V(k, hmi.Fs[h.fs[j]].vs[2]) * 1000;
					}
					intersect = NoDivTriTriIsect(V0, V1, V2, U0, U1, U2);
					if (intersect) break;
				}
			}
		}
		if (intersect) {
			mq.Self_In[h.id] = 1;
			inN++;
		}
	}
//	cout << "self intersected elements: " << inN <<" "<< inN * (1.0/hmi.Hs.size())<< endl;
	cout << inN << " " << inN * (1.0 / hmi.Hs.size()) << endl;
}
//========HYBRID===========//


