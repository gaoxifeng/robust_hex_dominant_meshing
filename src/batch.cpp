/*
    batch.cpp -- command line interface to Instant Meshes

    This file is part of the implementation of

        Instant Field-Aligned Meshes
        Wenzel Jakob, Daniele Panozzo, Marco Tarini, and Olga Sorkine-Hornung
        In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2015)

    All rights reserved. Use of this source code is governed by a
    BSD-style license that can be found in the LICENSE.txt file.
*/

#include "batch.h"
#include "meshio.h"
#include "hierarchy.h"
#include "optimizer.h"
#include "dedge.h"
#include "subdivide.h"
#include "timer.h"

void batch_process(char *input, char *output,
	uint32_t dimension, Float tlen, Float scale, int smooth_iter) {
	//cout << endl;
	//cout << "Running in batch mode:" << endl;
	//cout << "   Input file             = " << input << endl;
	//cout << "   Output file            = " << output << endl;

	MultiResolutionHierarchy mRes;
	Optimizer *mOptimizer;

	Timer<> timer;
	timer.beginStage("data pre-processing");
	mRes.load(input);
	return;
	MeshStats stats = mRes.compute_mesh_stats(mRes.F(), mRes.V(0));

	if (dimension ==2 && ((stats.mMaximumEdgeLength * 4 / stats.mAverageEdgeLength > scale || stats.mMaximumEdgeLength > stats.mAverageEdgeLength * 1.5))) {
		cout << "Input mesh is too coarse for the desired output edge length "
			"(max input mesh edge length=" << stats.mMaximumEdgeLength
			<< "), subdividing .." << endl;
		VectorXu V2E, E2E;
		VectorXb boundary, nonManifold;
		build_dedge(mRes.F(), mRes.V(0), V2E, E2E, boundary, nonManifold);
		Float left = scale * (Float)stats.mAverageEdgeLength / 4;
		Float right = (Float)stats.mAverageEdgeLength * 1.5;
		subdivide(mRes.F(), mRes.V(0), V2E, E2E, boundary, nonManifold, std::min(left, right));
		write_surface_mesh_OBJ(mRes.V(0), mRes.F(), output);
	}
	else if(dimension == 3){
		mRes.tElen_ratio = tlen;
		mRes.tet_meshing();
	}
	else return;

	mRes.build(); 

	//char path[1024];
	//sprintf(path, "%s%s", output, "_tet.num");
	//write_statistics_TXT(mRes.sta, path);
	//return;

	mRes.setScale(scale);
	timer.endStage();
	mRes.sta.timings.push_back(timer.value());

	timer.beginStage("rosy optimization");

	mOptimizer = new Optimizer(mRes);
	mOptimizer->setMaxIterations(smooth_iter);
	mOptimizer->setOptimizeOrientations(true);
	mOptimizer->notify();
	mOptimizer->wait();

	timer.endStage();
	mRes.sta.timings.push_back(timer.value());

	timer.beginStage("posy optimization");

	mOptimizer->setOptimizePositions(true);
	mOptimizer->notify();
	mOptimizer->wait();

	timer.endStage();
	mRes.sta.timings.push_back(timer.value());

	mOptimizer->shutdown();

	timer.beginStage("mesh extraction");

	mRes.re_color = true;
	mRes.splitting = true;
	mRes.decomposes = true;
	mRes.doublets = true;
	mRes.triangles = true;

	if (!mRes.tetMesh()) {
		mRes.re_color = true;
		mRes.splitting = true;
		mRes.decomposes = true;
		mRes.doublets = true;
		mRes.triangles = true;

		mRes.meshExtraction2D();
	}
	else {
		mRes.re_color = true;
		mRes.splitting = true;
		mRes.meshExtraction3D();//mRes.extractTet();
	}

	timer.endStage();
	mRes.sta.timings.push_back(timer.value());

	if (!mRes.tetMesh()) {
		char patho[1024];
		sprintf(patho, "%s%s", output, "_surout.obj");
		write_surface_mesh_OBJ(mRes.mV_tag, mRes.F_tag, patho);
		sprintf(output, "%s%s", output, "tri.obj");

		sprintf(patho, "%s%s", output, "_V_flag.txt");
		write_Vertex_Types_TXT(mRes.V_flag, patho);
	}
	else
	{
		char path[1024];
		sprintf(path, "%s%s", output, ".HYBRID");
		write_volume_mesh_HYBRID(mRes.mV_tag, mRes.F_tag, mRes.P_tag, mRes.Hex_flag, mRes.PF_flag, path);
		sprintf(path, "%s%s", output, "_statistics.txt");
		write_statistics_TXT(mRes.sta, path);


		//char vtk_mesh[300], mesh_mesh[300], obj_mesh[300];
		//sprintf(vtk_mesh, "%s%s", output, ".vtk");
		//sprintf(obj_mesh, "%s%s", output, ".obj");

		//write_surface_mesh_OBJ(mRes.mV_tag, mRes.F_tag, obj_mesh);
	}
	//else
	//{
	//	char vtk_mesh[300], mesh_mesh[300], obj_mesh[300];
	//	sprintf(vtk_mesh, "%s%s", output, ".vtk");
	//	sprintf(mesh_mesh, "%s%s", output, ".mesh");
	//	sprintf(obj_mesh, "%s%s", output, ".obj");

	//	write_volume_mesh_VTK(mPVs, mRes.mF_done_tagging, vtk_mesh);
	//	write_volume_mesh_MESH(mPVs, mRes.mF_done_tagging, mesh_mesh);
	//	write_surface_mesh_OBJ(mPVsB, Fs_boundary, obj_mesh);
	//}
	//system("PAUSE");
}
