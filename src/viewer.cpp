#include "viewer.h"
#include "im_resources.h"
#include "timer.h"
#include "bvh.h"
#include <nanogui/serializer/opengl.h>

Viewer::Viewer(std::string &filename, bool fullscreen)
    : Screen(Vector2i(1280, 960), "Instant Hex-Dominant Meshes", true),
      mOptimizer(nullptr) {

    /* Load the input mesh */
	if (!filename.empty())
		;// mRes.load(filename);

    mOptimizer = new Optimizer(mRes);

    /* Initialize shaders for rendering geometry and fields */
    mMeshShader.init("mesh_shader",
        (const char *) shader_mesh_vert,
        (const char *) shader_mesh_frag,
        (const char *) shader_mesh_geo);

    mTetShader.init("tet_shader",
        (const char *) shader_tet_vert,
        (const char *) shader_mesh_frag,
        (const char *) shader_tet_geo);

    mOrientationFieldShaderTet.init("orientation_field_shader_tet",
        (const char *) shader_orientation_field_tet_vert,
        (const char *) shader_orientation_field_tet_frag,
        (const char *) shader_orientation_field_tet_geo);

    mOrientationFieldShaderTri.init("orientation_field_shader_tri",
        (const char *) shader_orientation_field_tri_vert,
        (const char *) shader_orientation_field_tri_frag,
        (const char *) shader_orientation_field_tri_geo);

    mPositionFieldShader.init("position_field_shader",
        (const char *) shader_position_field_vert,
        (const char *) shader_position_field_frag);

    mOrientationSingularityShaderTet.init("orientation_singularity_shader_tet",
        (const char *) shader_singularity_tet_vert,
        (const char *) shader_singularity_tet_frag);

    mOrientationSingularityShaderTri.init("orientation_singularity_shader_tri",
        (const char *) shader_singularity_tri_vert,
        (const char *) shader_singularity_tri_frag,
        (const char *) shader_singularity_tri_geo);

    mPositionSingularityShaderTet.init("position_singularity_shader_tet",
        (const char *) shader_singularity_tet_vert,
        (const char *) shader_singularity_tet_frag);

    mPositionSingularityShaderTri.init("position_singularity_shader_tri",
        (const char *) shader_singularity_tri_vert,
        (const char *) shader_singularity_tri_frag,
        (const char *) shader_singularity_tri_geo);

    mExtractionResultShader.init("extraction_result",
        (const char *) shader_singularity_tet_vert,
        (const char *) shader_singularity_tet_frag);
	mExtractionResultShader2.init("extraction_result2",
		(const char *)shader_singularity_tet_vert,
		(const char *)shader_singularity_tet_frag);

	
	mEdge_color_morph2.init("mEdge_color_morph2",
		(const char *)shader_singularity_tet_vert,
			(const char *)shader_singularity_tet_frag);


	mExtractionResultShader_E_tagging.init("Shader_E_tagging",
		(const char *)shader_singularity_tet_vert,
		(const char *)shader_singularity_tet_frag);
	mExtractionResultShader_E_degeneracy.init("Shader_E_tagging",
		(const char *)shader_singularity_tet_vert,
		(const char *)shader_singularity_tet_frag);
	mExtractionResultShader_E_triangle.init("Shader_E_tagging",
		(const char *)shader_singularity_tet_vert,
		(const char *)shader_singularity_tet_frag);
	mExtractionResultShader_E_done.init("Shader_E_local",
		(const char *)shader_singularity_tet_vert,
		(const char *)shader_singularity_tet_frag);

	mExtractionResultShader_F_done_local.init("Shader_F_tagging",
		(const char *)shader_mesh_vert,
		(const char *)shader_mesh_frag,
		(const char *)shader_mesh_geo);
	mExtractionResultShader_F_done.init("Shader_F_local",
		(const char *)shader_mesh_vert,
		(const char *)shader_mesh_frag,
		(const char *)shader_mesh_geo);

    /* Default view setup */
    mCamera.arcball = Arcball();
    mCamera.arcball.setSize(mSize);
    mCamera.modelTranslation = -mRes.aabb().center().cast<float>();
    mCamera.modelZoom = 3.0f / mRes.aabb().extents().cwiseAbs().maxCoeff();
    mCamera.zoom = 1.0f;
    mLightPosition = Vector3f(0.0f, 0.3f, 5.0f);
    mBaseColor = Vector4f(0.4f, 0.5f, 0.7f, 1.f);
    mBaseColorBoundary = mRes.tetMesh() ? Vector4f(0.0f, 0.0f, 1.0f, .2f) : mBaseColor;
    mSpecularColor = Vector4f(1.f, 1.f, 1.f, 1.f);
    mSpecularColorBoundary = mRes.tetMesh() ? Vector4f(1.f, 1.f, 1.f, .2f) : mSpecularColor;
    mTranslate = false;

	/* Scan over example files in the 'datasets' directory */
	auto ctx = nvgContext();
	try {
		mExampleImages = nanogui::loadImageDirectory(ctx, "resources");
	}
	catch (const std::runtime_error &e) {
		cout << "Unable to load image data: " << e.what() << endl;
	}
	mExampleImages.insert(mExampleImages.begin(),
		std::make_pair(nvgImageIcon(ctx, loadmesh), ""));

	/* Initialize user interface */
    Window *window = new Window(this, "Instant 2D/3D Meshes");
    window->setPosition(Vector2i(15, 15));
    window->setLayout(new GroupLayout());

	PopupButton *openBtn0 = new PopupButton(window, "Open mesh");
	openBtn0->setBackgroundColor(Color(0, 255, 0, 25));
	openBtn0->setIcon(ENTYPO_ICON_FOLDER);
	Popup *popup0 = openBtn0->popup();
	VScrollPanel *vscroll = new VScrollPanel(popup0);
	ImagePanel *panel0 = new ImagePanel(vscroll);
	panel0->setImages(mExampleImages);
	panel0->setCallback([&, openBtn0](int i) {
		openBtn0->setPushed(false);

		std::string filename2 = mExampleImages[i].second;
		std::string extension;
		if (filename2.size() > 4)
			extension = str_tolower(filename2.substr(filename2.size() - 4));

		if (filename2.empty()) {
			filename2 = nanogui::file_dialog({
				{ "obj", "Wavefront OBJ" },
				{ "vtk", "Visualization Toolkit VTk" },
				{ "HYBRID", "general polyhedral mesh" }
			}, false);
			if (filename2 == "")
				return;
		}
		else if (extension != ".ply" && extension != ".obj" && extension != ".aln")
			;// filename2 = filename2 + ".ply";

		mRes.load(filename2);
		filename = filename2;
		mRes.outpath = filename2;
		/* Default view setup */
		mCamera.arcball = Arcball();
		mCamera.arcball.setSize(mSize);
		mCamera.modelTranslation = -mRes.aabb().center().cast<float>();
		mCamera.modelZoom = 3.0f / mRes.aabb().extents().cwiseAbs().maxCoeff();
		mCamera.zoom = 1.0f;
		mLightPosition = Vector3f(0.0f, 0.3f, 5.0f);
		mBaseColor = Vector4f(0.4f, 0.5f, 0.7f, 1.f);
		//mBaseColor = Vector4f(1.0f, 1.0f, 1.0f, 1.f);
		mBaseColorBoundary = mRes.tetMesh() ? Vector4f(0.0f, 0.0f, 1.0f, .2f) : mBaseColor;
		mSpecularColor = Vector4f(1.f, 1.f, 1.f, 1.f);
		mSpecularColorBoundary = mRes.tetMesh() ? Vector4f(1.f, 1.f, 1.f, .2f) : mSpecularColor;
		mTranslate = false;

	});

	if (mRes.mV[0].cols()) {
		/* Default view setup */
		mCamera.arcball = Arcball();
		mCamera.arcball.setSize(mSize);
		mCamera.modelTranslation = -mRes.aabb().center().cast<float>();
		mCamera.modelZoom = 3.0f / mRes.aabb().extents().cwiseAbs().maxCoeff();
		mCamera.zoom = 1.0f;
		mLightPosition = Vector3f(0.0f, 0.3f, 5.0f);
		mBaseColor = Vector4f(0.4f, 0.5f, 0.7f, 1.f);
		//mBaseColor = Vector4f(1.0f, 1.0f, 1.0f, 1.f);
		mBaseColorBoundary = mRes.tetMesh() ? Vector4f(0.0f, 0.0f, 1.0f, .2f) : mBaseColor;
		mSpecularColor = Vector4f(1.f, 1.f, 1.f, 1.f);
		mSpecularColorBoundary = mRes.tetMesh() ? Vector4f(1.f, 1.f, 1.f, .2f) : mSpecularColor;
		mTranslate = false;
	}

    PopupButton *openBtn = new PopupButton(window, "Layers");
    auto popup = openBtn->popup();
    popup->setLayout(new GroupLayout());

    new Label(popup, "Render layers", "sans-bold");
    mLayers[Layers::Tetrahedra]               = new CheckBox(popup, "Tetrahedra");
    mLayers[Layers::OrientationField]         = new CheckBox(popup, "Orientation field");
    mLayers[Layers::OrientationSingularities] = new CheckBox(popup, "Orientation singularities");
    mLayers[Layers::PositionField]            = new CheckBox(popup, "Position field");
    mLayers[Layers::PositionSingularities]    = new CheckBox(popup, "Position singularities");
    mLayers[Layers::Boundary]                 = new CheckBox(popup, "Boundary");
    mLayers[Layers::BoundaryWireframe]        = new CheckBox(popup, "Boundary wireframe");
    mLayers[Layers::FaceLabels]               = new CheckBox(popup, "Face Labels");
	mLayers[Layers::Face_afterLabels]		  = new CheckBox(popup, "ExtractedFace Labels");
    mLayers[Layers::VertexLabels]             = new CheckBox(popup, "Vertex Labels");
    mLayers[Layers::ExtractionResult]         = new CheckBox(popup, "Extraction result (colors)");
    mLayers[Layers::ExtractionResult2]        = new CheckBox(popup, "Extraction result (faces)");

    int ctr = 0;
    for (auto l : mLayers) {
        l->setChecked(false);
        l->setId("layer." + std::to_string(ctr++));
    }
    //mLayers[Layers::Boundary]->setChecked(true);
    //mLayers[Layers::PositionField]->setChecked(true);

    new Label(window, "Flags", "sans-bold");

    mAlignmentBox = new CheckBox(window, "Boundary alignment");
    mAlignmentBox->setChecked(mOptimizer->alignment());
    mAlignmentBox->setId("alignment");

    mExtrinsicBox = new CheckBox(window, "Extrinsic smoothing");
    mExtrinsicBox->setChecked(mOptimizer->extrinsic());
    mExtrinsicBox->setId("extrinsic");
    mExtrinsicBox->setEnabled(!mRes.tetMesh());

    mRandomizationBox = new CheckBox(window, "Randomization");
    mRandomizationBox->setChecked(mOptimizer->randomization());
    mRandomizationBox->setId("randomization");
    mHierarchyBox = new CheckBox(window, "Hierarchy");
    mHierarchyBox->setChecked(mOptimizer->hierarchy());
    mHierarchyBox->setId("hierarchy");

	new Label(window, "Tet edge-len", "sans-bold");
	mtElenRatioBox = new FloatBox<Float>(window);
	mtElenRatioBox->setValue(mRes.tet_elen_ratio());
	mtElenRatioBox->setEditable(true);
	mtElenRatioBox->setAlignment(TextBox::Alignment::Right);
	mtElenRatioBox->setId("tElen_ratio");

	mTmeshingBtn = new Button(window, "Tet-meshing", ENTYPO_ICON_FLASH);
	mTmeshingBtn->setBackgroundColor(Color(0, 0, 255, 25));
	mTmeshingBtn->setFlags(Button::Flags::ToggleButton);
	mTmeshingBtn->setChangeCallback([&](bool value) {
		mRes.tet_meshing();
	});

	new Label(window, "Output scale", "sans-bold");
	mScaleBox = new FloatBox<Float>(window);
	mScaleBox->setValue(mRes.scale());
	mScaleBox->setEditable(true);
	mScaleBox->setAlignment(TextBox::Alignment::Right);
	mScaleBox->setId("outputscale");

    new Label(window, "Iterations per level", "sans-bold");
    mIterationBox = new IntBox<uint32_t>(window);
    mIterationBox->setValue(mOptimizer->maxIterations());
    mIterationBox->setEditable(true);
    mIterationBox->setAlignment(TextBox::Alignment::Right);
    mIterationBox->setId("iterations");

	
	mSolveDatastructureBtn = new Button(window, "Build-Structure", ENTYPO_ICON_FLASH);
	mSolveDatastructureBtn->setBackgroundColor(Color(0, 0, 255, 25));
	mSolveDatastructureBtn->setFlags(Button::Flags::ToggleButton);
	mSolveDatastructureBtn->setChangeCallback([&](bool value) {
		mRes.build();

		mTetShader.bind();

		MatrixXf vertexColors = MatrixXf::Zero(4, mRes.vertexCount());
		mTetShader.uploadAttrib("position", mRes.V());
		mTetShader.uploadIndices(mRes.T());
		mTetShader.uploadAttrib("color", vertexColors);

		mMeshShader.bind();
		mMeshShader.shareAttrib(mTetShader, "position");
		mMeshShader.uploadIndices(mRes.F());
		mMeshShader.uploadAttrib("normal", mRes.N());

		if (mRes.tetMesh()) {
			mOrientationFieldShaderTet.bind();
			mOrientationFieldShaderTet.shareAttrib(mTetShader, "position");
			mOrientationFieldShaderTet.uploadAttrib("q", mRes.Q());
		}
		else {
			mOrientationFieldShaderTri.bind();
			mOrientationFieldShaderTri.shareAttrib(mTetShader, "position");
			mOrientationFieldShaderTri.uploadAttrib("q", mRes.Q());
			mOrientationFieldShaderTri.uploadAttrib("n", mRes.N());
		}

		mPositionFieldShader.bind();
		mPositionFieldShader.uploadAttrib("o", mRes.O());

		mLayers[Layers::Boundary]->setChecked(true);
		mLayers[Layers::PositionField]->setChecked(true);
	});
	

    mSolveOrientationBtn = new Button(window, "Rosy", ENTYPO_ICON_FLASH);
    mSolveOrientationBtn->setBackgroundColor(Color(0, 0, 255, 25));
    mSolveOrientationBtn->setFlags(Button::Flags::ToggleButton);
    mSolveOrientationBtn->setChangeCallback([&](bool value) {
        mOptimizer->setOptimizeOrientations(value);
        mOptimizer->notify();

        //mLayers[Layers::OrientationField]->setChecked(true);
        mLayers[Layers::OrientationSingularities]->setChecked(true);
        mLayers[Layers::PositionField]->setChecked(false);
        mLayers[Layers::PositionSingularities]->setChecked(false);
        if (value == false)
            updateOrientationSingularities();
    });

    mSolvePositionBtn = new Button(window, "Posy", ENTYPO_ICON_FLASH);
    mSolvePositionBtn->setBackgroundColor(Color(0, 0, 255, 25));
    mSolvePositionBtn->setFlags(Button::Flags::ToggleButton);
    mSolvePositionBtn->setChangeCallback([&](bool value) {
        if (mRes.combed()) {
            mHierarchyBox->setChecked(false);
            mOptimizer->setHierarchy(false);
        }
        mOptimizer->setOptimizePositions(value);
        mOptimizer->notify();

        mLayers[Layers::OrientationField]->setChecked(false);
        mLayers[Layers::OrientationSingularities]->setChecked(false);
        mLayers[Layers::PositionField]->setChecked(true);
        mLayers[Layers::PositionSingularities]->setChecked(true);
        if (value == false)
            updatePositionSingularities();
    });

	mEdgeTagging_done = false;
	mEdgeTaggingBtn = new Button(window, "Coloring", ENTYPO_ICON_FLASH);
	mEdgeTaggingBtn->setBackgroundColor(Color(0, 0, 255, 25));
	mEdgeTaggingBtn->setFlags(Button::Flags::ToggleButton);
	mEdgeTaggingBtn->setChangeCallback([&](bool value) {
		if (!mRes.tetMesh())
			mRes.init_edge_tagging2D();
		else
		{
			mRes.init_edge_tagging3D();
			//mRes.projectBack3D2();
		}
		mEdgeTagging_done = true;
		auto const &R = mRes.E_rend;
		mExtractionResultShader.bind();
		mExtractionResultShader.uploadAttrib("position", MatrixXf(R.block(0, 0, 3, R.cols())));
		mExtractionResultShader.uploadAttrib("color", MatrixXf(R.block(3, 0, 3, R.cols())));
	});
	new Label(window, "Morphing", "sans-bold");
	Slider *slider2 = new Slider(window);
	slider2->setValue(0.0);
	auto cb = [&](Float value) {
		mRes.E_I_rend = (1 - value) * mRes.E_rend + value*mRes.E_O_rend;
	};
	cb(0.0f);
	slider2->setCallback(cb);
	slider2->setId("slider2");

	Slider *slider_morph2 = new Slider(window);
	slider_morph2->setValue(0.0);
	auto cb_morph2 = [&](Float value) {
		mRes.E_I_rend_o = (1 - value) * mRes.E_rend_o + value*mRes.E_O_rend_o;
	};
	cb_morph2(0.0f);
	slider_morph2->setCallback(cb_morph2);
	slider_morph2->setId("slider_morph2");

	PopupButton *openBtn2 = new PopupButton(window, "Extractionlayers");
	auto popup2 = openBtn2->popup();
	popup2->setLayout(new GroupLayout());
	new Label(popup2, "Extractionlayers", "sans-bold");
	mExtractLayers[Extraction_Condition::ReColor] = new CheckBox(popup2, "ReColor");
	mExtractLayers[Extraction_Condition::Quadric] = new CheckBox(popup2, "Quadric");
	mExtractLayers[Extraction_Condition::Splitting] = new CheckBox(popup2, "Splitting");
	mExtractLayers[Extraction_Condition::Triangles] = new CheckBox(popup2, "Triangles");
	mExtractLayers[Extraction_Condition::Doublets] = new CheckBox(popup2, "Doublets");
	mExtractLayers[Extraction_Condition::Decompose] = new CheckBox(popup2, "Decompose");

	int ctr2 = 0;
	for (auto l : mExtractLayers) {
		l->setChecked(true);
		l->setId("layer." + std::to_string(ctr2++));
	}
	mExtractLayers[Extraction_Condition::Quadric]->setChecked(false);

	//mReColorBox = new CheckBox(window, "ReColor");
	//mReColorBox->setChecked(false);
	//mReColorBox->setId("ReColor");

	//mQuadricBox = new CheckBox(window, "Quadric");
	//mQuadricBox->setChecked(false);
	//mQuadricBox->setId("Quadric");

	//mSplitBox = new CheckBox(window, "splitting");
	//mSplitBox->setChecked(false);
	//mSplitBox->setId("splitting");

	//mTrianglesBox = new CheckBox(window, "Triangles");
	//mTrianglesBox->setChecked(false);
	//mTrianglesBox->setId("Triangles");

	//mDoubletsBox = new CheckBox(window, "Doublets");
	//mDoubletsBox->setChecked(false);
	//mDoubletsBox->setId("Doublets");
	//
	//mDecomposeBox = new CheckBox(window, "Decompose");
	//mDecomposeBox->setChecked(false);
	//mDecomposeBox->setId("Decompose");
	
	
    mExtractBtn = new Button(window, "Extract", ENTYPO_ICON_FLASH);
    mExtractBtn->setBackgroundColor(Color(0, 255, 0, 25));
    mExtractBtn->setCallback([&]() {
		if (!mRes.tetMesh()) {
			//mRes.extractTri();
			if (mRes.global_parameterization) mRes.meshExtraction2D_global();
			else  mRes.meshExtraction2D();

			mExtractionResultShader_F_done.bind();
			mExtractionResultShader_F_done.uploadAttrib("position", mRes.mV_final_rend);
			mExtractionResultShader_F_done.uploadIndices(mRes.F_final_rend);
			mShow_F_done->setChecked(true);

			auto const &R4 = mRes.E_final_rend;
			mExtractionResultShader_E_done.bind();
			mExtractionResultShader_E_done.uploadAttrib("position", MatrixXf(R4.block(0, 0, 3, R4.cols())));
			mExtractionResultShader_E_done.uploadAttrib("color", MatrixXf(R4.block(3, 0, 3, R4.cols())));
			mShow_E_done->setChecked(true);

			auto const &R2 = mRes.E_tag_rend;
			mExtractionResultShader_E_tagging.bind();
			mExtractionResultShader_E_tagging.uploadAttrib("position", MatrixXf(R2.block(0, 0, 3, R2.cols())));
			mExtractionResultShader_E_tagging.uploadAttrib("color", MatrixXf(R2.block(3, 0, 3, R2.cols())));
			//mShow_E_tagging->setChecked(true);

			mLayers[PositionSingularities]->setChecked(false);
			mLayers[Layers::PositionField]->setChecked(false);
			mLayers[Layers::Boundary]->setChecked(false);
		}
		else
		{
			//mRes.extractTet();
			mRes.meshExtraction3D();

			auto const &R2 = mRes.E_tag_rend;
			mExtractionResultShader_E_tagging.bind();
			mExtractionResultShader_E_tagging.uploadAttrib("position", MatrixXf(R2.block(0, 0, 3, R2.cols())));
			mExtractionResultShader_E_tagging.uploadAttrib("color", MatrixXf(R2.block(3, 0, 3, R2.cols())));
			mShow_E_tagging->setChecked(true);
		}

		mExtractionResultShader_F_done_local.bind();
		mExtractionResultShader_F_done_local.uploadAttrib("position", mRes.mV_tag_rend);
		mExtractionResultShader_F_done_local.uploadIndices(mRes.F_tag_rend);
		//mShow_F_local->setChecked(true);



		//auto const &R3 = mRes.E_tag_left_rend;
		//mExtractionResultShader_E_degeneracy.bind();
		//mExtractionResultShader_E_degeneracy.uploadAttrib("position", MatrixXf(R3.block(0, 0, 3, R3.cols())));
		//mExtractionResultShader_E_degeneracy.uploadAttrib("color", MatrixXf(R3.block(3, 0, 3, R3.cols())));
		//mShow_E_degeneracy->setChecked(true);

	});

    new Label(window, "Slicing plane", "sans-bold");
    Slider *slider = new Slider(window);
    mSplit = Vector4f(1.f, 0.f, 0.f, mRes.aabb().center().x());
    slider->setValue(0.0);
    auto cb2 = [&](Float value) {
		float offset = (mRes.aabb().max.x() - mRes.aabb().min.x()) * 0.5;
        mSplit.w() = -((1-value) * (mRes.aabb().min.x() - offset) + value * (mRes.aabb().max.x() + offset));
    };
    cb2(0.0f);
    slider->setCallback(cb2);
    slider->setId("slider1");

    Widget *panel = new Widget(window);
    panel->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 10));

    mJumpBox = new IntBox<uint32_t>(panel);
    mJumpButton = new Button(panel, "Go");
    mJumpBox->setEditable(true);
    mJumpBox->setFixedWidth(90);

    mJumpButton->setCallback([&]{
        uint32_t v = mJumpBox->value();
        if (v >= mRes.V().cols())
            return;
        Vector4f p, n;
        p << mRes.V().col(v), 1.f;
        n << mRes.N().col(v), 0.f;

        Vector3f shift = -mCamera.modelTranslation - p.head<3>();
        mCamera.modelTranslation += shift;

        Eigen::Matrix4f model, view, proj;
        computeCameraMatrices(model, view, proj);

        mCamera.eye = (model * n).head<3>().normalized() * 5;
        mLightPosition = mCamera.eye + Vector3f::UnitX();
    });

    performLayout();


	Window *window_debug = new Window(this, "Debug Window");
	window_debug ->setPosition(Vector2i(210, 400));
	window_debug ->setLayout(new GroupLayout());

	new Label(window_debug, "Face Id", "sans-bold");
	mFaceIdBox = new IntBox<int32_t>(window_debug);
	mFaceIdBox->setValue(-1);
	mFaceIdBox->setEditable(true);
	mFaceIdBox->setAlignment(TextBox::Alignment::Right);
	mFaceIdBox->setId("faceid");

	new Label(window_debug, "Vertex Id", "sans-bold");
	mVertexIdBox = new IntBox<int32_t>(window_debug);
	mVertexIdBox->setValue(-1);
	mVertexIdBox->setEditable(true);
	mVertexIdBox->setAlignment(TextBox::Alignment::Right);
	mVertexIdBox->setId("vertexid");


	mMorph2Box = new CheckBox(window_debug, "edge-color-o");
	mMorph2Box->setChecked(false);
	mMorph2Box->setId("Morph_edge2");

	new Label(window_debug, "Edit tagging", "sans-bold");

	mShow_F_local = new CheckBox(window_debug, "Show_F_local");
	mShow_F_local->setId("showdonelocalF");
		mShow_E_tagging = new CheckBox(window_debug, "Show_E_tagging");
		mShow_E_tagging->setId("showdonetagging");
		mShow_E_degeneracy = new CheckBox(window_debug, "Show_E_degeneracy");
		mShow_E_degeneracy->setId("showdonedegeneracy");
		mShow_E_triangle = new CheckBox(window_debug, "Show_E_triangle");
		mShow_E_triangle->setId("showdonetriangle");

	new Label(window_debug, "Final", "sans-bold");
	mShow_F_done = new CheckBox(window_debug, "Show_F_done");
	mShow_F_done->setId("showdoneF");
		mShow_E_done = new CheckBox(window_debug, "Show_E_done");
		mShow_E_done->setId("showdoneE");

	new Label(window, "Output", "sans-bold");
	mOutputBtn = new Button(window_debug, "Output", ENTYPO_ICON_FLASH);
	mOutputBtn->setBackgroundColor(Color(0, 255, 0, 25));
	mOutputBtn->setCallback([&]() {
		char patho[300];
		if (!mRes.tetMesh()) {
			//sprintf(patho, "%s%s", filename.c_str(), "_surout.vtk");
			//write_surface_mesh_VTK(mRes.mV_tag, mRes.F_tag, patho);
			//write_surface_mesh_VTK(mRes.mV_final, mRes.mF_done_triangle, "../datasets/out_done_triangle.vtk");
			sprintf(patho, "%s%s", filename.c_str(), "_surout.obj");
			write_surface_mesh_OBJ(mRes.mV_tag, mRes.F_tag, patho);

			sprintf(patho, "%s%s", filename.c_str(), "_V_flag.txt");
			write_Vertex_Types_TXT(mRes.V_flag, patho);
		}
		else{
			sprintf(patho, "%s%s", filename.c_str(), "_volout.obj");
			write_surface_mesh_OBJ(mRes.mV_tag, mRes.F_tag, patho);
			sprintf(patho, "%s%s", filename.c_str(), "_volout.vtk");
			write_volume_mesh_VTK(mRes.mV_tag, mRes.Es_reddash_left, mRes.F_tag, mRes.F_tag_type, patho);
			//write_volume_mesh_MESH(mPVs, mRes.F_final, "../datasets/out_done_tagging_vol.mesh");
		}
	});

	performLayout();
}

Viewer::~Viewer() {
    mOptimizer->shutdown();
    delete mOptimizer;
}

bool Viewer::mouseMotionEvent(const Vector2i &p, const Vector2i &rel,
                              int button, int modifiers) {
    if (!Screen::mouseMotionEvent(p, rel, button, modifiers)) {
        if (mCamera.arcball.motion(p)) {
            //repaint();
        } else if (mTranslate) {
            Eigen::Matrix4f model, view, proj;
            computeCameraMatrices(model, view, proj);
            float zval = project(mRes.aabb().center().cast<float>(), view * model, proj, mSize).z();
            Eigen::Vector3f pos1 =
                unproject(Eigen::Vector3f(p.x(), mSize.y() - p.y(), zval),
                          view * model, proj, mSize);
            Eigen::Vector3f pos0 = unproject(
                Eigen::Vector3f(mTranslateStart.x(),
                                mSize.y() - mTranslateStart.y(), zval),
                view * model, proj, mSize);
            mCamera.modelTranslation =
                mCamera.modelTranslation_start + (pos1 - pos0);
            //repaint();
        }
    }
    return true;
}

bool Viewer::mouseButtonEvent(const Vector2i &p, int button, bool down, int modifiers) {
    if (!Screen::mouseButtonEvent(p, button, down, modifiers)) {
        if (button == GLFW_MOUSE_BUTTON_1 && modifiers == 0) {
            mCamera.arcball.button(p, down);
        } else if (button == GLFW_MOUSE_BUTTON_2 ||
                   (button == GLFW_MOUSE_BUTTON_1 && modifiers == GLFW_MOD_SHIFT)) {
            mCamera.modelTranslation_start = mCamera.modelTranslation;
            mTranslate = true;
            mTranslateStart = p;
        }
    }
    if (button == GLFW_MOUSE_BUTTON_1 && !down)
        mCamera.arcball.button(p, false);
    if (!down) {
        mTranslate = false;
    }
    return true;
}

bool Viewer::resizeEvent(const Vector2i &size) {
    mCamera.arcball.setSize(mSize);
    return true;
}

bool Viewer::scrollEvent(const Vector2i &p, const Eigen::Vector2f &rel) {
    if (!Screen::scrollEvent(p, rel)) {
        mCamera.zoom = std::max(0.1, mCamera.zoom * (rel.y() > 0 ? 1.1 : 0.9));
        //repaint();
    }
    return true;
}

void Viewer::updatePositionSingularities() {
    if (mRes.tetMesh()) {
        mRes.detectPositionSingularitiesTet();
        auto const &S = mRes.positionSingularities();
        mPositionSingularityShaderTet.bind();
        mPositionSingularityShaderTet.uploadAttrib("position", MatrixXf(S.block(0, 0, 3, S.cols())));
        mPositionSingularityShaderTet.uploadAttrib("color",    MatrixXf(S.block(3, 0, 3, S.cols())));
    } else {
        mRes.detectPositionSingularitiesTri();
        auto const &S = mRes.positionSingularities();
        mPositionSingularityShaderTri.bind();
        mPositionSingularityShaderTri.uploadAttrib("position", MatrixXf(S.block(0, 0, 3, S.cols())));
        mPositionSingularityShaderTri.uploadAttrib("normal",   MatrixXf(S.block(3, 0, 3, S.cols())));
        mPositionSingularityShaderTri.uploadAttrib("color",    MatrixXf(S.block(6, 0, 3, S.cols())));
    }
}

void Viewer::updateOrientationSingularities() {
    if (mRes.tetMesh()) {
        mRes.detectOrientationSingularitiesTet();
        auto const &S = mRes.orientationSingularities();
        mOrientationSingularityShaderTet.bind();
        mOrientationSingularityShaderTet.uploadAttrib("position", MatrixXf(S.block(0, 0, 3, S.cols())));
        mOrientationSingularityShaderTet.uploadAttrib("color",    MatrixXf(S.block(3, 0, 3, S.cols())));
    } else {
        mRes.detectOrientationSingularitiesTri();
        auto const &S = mRes.orientationSingularities();
        mOrientationSingularityShaderTri.bind();
        mOrientationSingularityShaderTri.uploadAttrib("position", MatrixXf(S.block(0, 0, 3, S.cols())));
        mOrientationSingularityShaderTri.uploadAttrib("normal",   MatrixXf(S.block(3, 0, 3, S.cols())));
        mOrientationSingularityShaderTri.uploadAttrib("color",    MatrixXf(S.block(6, 0, 3, S.cols())));
    }
}

void Viewer::drawContents() {
	glClearColor(.5, .5, .5, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    mOptimizer->setAlignment(mAlignmentBox->checked());
    mOptimizer->setRandomization(mRandomizationBox->checked());
    mOptimizer->setHierarchy(mHierarchyBox->checked());
    mOptimizer->setMaxIterations(mIterationBox->value());
	mRes.setScale(mScaleBox->value());
	mRes.set_tet_elen_ratio(mtElenRatioBox->value());

	mRes.re_color = mExtractLayers[Extraction_Condition::ReColor]->checked();
	mRes.Qquadric = mExtractLayers[Extraction_Condition::Quadric]->checked();
	mRes.doublets = mExtractLayers[Extraction_Condition::Doublets]->checked();
	mRes.splitting = mExtractLayers[Extraction_Condition::Splitting]->checked();
	mRes.triangles= mExtractLayers[Extraction_Condition::Triangles]->checked();
	mRes.decomposes = mExtractLayers[Extraction_Condition::Decompose]->checked();

	if (!mOptimizer->active()) {
        if (mSolveOrientationBtn->pushed()) {
            mSolveOrientationBtn->setPushed(false);
            updateOrientationSingularities();
        }
        if (mSolvePositionBtn->pushed()) {
            mSolvePositionBtn->setPushed(false);
            updatePositionSingularities();
        }
    } else if (!mOptimizer->hierarchy()) {
        if (mSolveOrientationBtn->pushed())
            updateOrientationSingularities();
    }
	if (mTmeshingBtn->pushed()) {
		mTmeshingBtn->setPushed(false);
	}
	if (mSolveDatastructureBtn->pushed()) {
		mSolveDatastructureBtn->setPushed(false);
	}
	if (mEdgeTaggingBtn->pushed()) {
		mEdgeTaggingBtn->setPushed(false);
	}
    Eigen::Matrix4f model, view, proj;
    computeCameraMatrices(model, view, proj);
    Eigen::Matrix4f mvp = proj * view * model;
    Eigen::Vector4f civ =
        (view * model).inverse() * Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f);

    // XXX improve this..
    if (mRes.tetMesh()) {
        mOrientationFieldShaderTet.bind();
        mOrientationFieldShaderTet.uploadAttrib("q", mRes.Q());
    } else {
        mOrientationFieldShaderTri.bind();
        mOrientationFieldShaderTri.uploadAttrib("q", mRes.Q());
    }

    mPositionFieldShader.bind();
    mPositionFieldShader.uploadAttrib("o", mRes.O());

	if (mEdgeTagging_done) {
		mExtractionResultShader2.bind();
		mExtractionResultShader2.uploadAttrib("position", MatrixXf(mRes.E_I_rend.block(0, 0, 3, mRes.E_I_rend.cols())));
		mExtractionResultShader2.uploadAttrib("color", MatrixXf(mRes.E_I_rend.block(3, 0, 3, mRes.E_I_rend.cols())));
	}
	if (mMorph2Box->checked()) {
		mEdge_color_morph2.bind();
		mEdge_color_morph2.uploadAttrib("position", MatrixXf(mRes.E_I_rend_o.block(0, 0, 3, mRes.E_I_rend_o.cols())));
		mEdge_color_morph2.uploadAttrib("color", MatrixXf(mRes.E_I_rend_o.block(3, 0, 3, mRes.E_I_rend_o.cols())));
	}


    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_BLEND);

    if (mLayers[Tetrahedra]->checked()) {
        mTetShader.bind();
        mTetShader.setUniform("light_position", mLightPosition);
        mTetShader.setUniform("model", model);
        mTetShader.setUniform("view", view);
        mTetShader.setUniform("proj", proj);
        mTetShader.setUniform("base_color", mBaseColor);
        //mTetShader.setUniform("specular_color", mSpecularColor);
        mTetShader.setUniform("split", mSplit);
        //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        mTetShader.drawIndexed(GL_LINES_ADJACENCY, 0, mRes.tetCount() * 4);
        //glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    glPointSize(1.5);

    if (mLayers[OrientationField]->checked()) {
        auto &shader = mRes.tetMesh() ? mOrientationFieldShaderTet : mOrientationFieldShaderTri;
        shader.bind();
        shader.setUniform("mvp", mvp);
        shader.setUniform("split", mSplit, false);
        shader.setUniform("scale", mRes.averageEdgeLength() / 3);
        shader.drawArray(GL_POINTS, 0, mRes.tetMesh() ? mRes.mO[0].cols() : mRes.faceCount());
    }

    if (mLayers[PositionField]->checked()) {
        mPositionFieldShader.bind();
        mPositionFieldShader.setUniform("mvp", mvp);
        mPositionFieldShader.setUniform("split", mSplit, false);
        mPositionFieldShader.drawArray(GL_POINTS, 0, mRes.vertexCount());
    }

    if (mLayers[OrientationSingularities]->checked()) {
        auto &shader = mRes.tetMesh() ? mOrientationSingularityShaderTet : mOrientationSingularityShaderTri;
        shader.bind();
        shader.setUniform("split", mSplit, false);
        shader.setUniform("mvp", mvp);
        shader.setUniform("scale", mRes.averageEdgeLength(), false);
        shader.drawArray(mRes.tetMesh() ? GL_LINES : GL_POINTS, 0, mRes.orientationSingularities().cols());
    }

    if (mLayers[PositionSingularities]->checked()) {
        auto &shader = mRes.tetMesh() ? mPositionSingularityShaderTet : mPositionSingularityShaderTri;
        shader.bind();
        shader.setUniform("split", mSplit, false);
        shader.setUniform("mvp", mvp);
        shader.setUniform("scale", mRes.averageEdgeLength(), false);
        shader.drawArray(mRes.tetMesh() ? GL_LINES : GL_POINTS, 0, mRes.positionSingularities().cols());
    }

    if (mLayers[Boundary]->checked()) {
        if (mRes.tetMesh()) {
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }
        mMeshShader.bind();
        mMeshShader.setUniform("light_position", mLightPosition);
        mMeshShader.setUniform("model", model);
        mMeshShader.setUniform("view", view);
        mMeshShader.setUniform("proj", proj);
        mMeshShader.setUniform("base_color", mBaseColorBoundary);
        //mMeshShader.setUniform("specular_color", mSpecularColorBoundary);
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0, 1.0);

        if (mRes.tetMesh()) {
            glDepthFunc(GL_LEQUAL);
            glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
            mMeshShader.drawIndexed(GL_TRIANGLES, 0, mRes.faceCount());
            glDepthFunc(GL_EQUAL);
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
            mMeshShader.drawIndexed(GL_TRIANGLES, 0, mRes.faceCount());
            glDepthFunc(GL_LEQUAL);
        } else {
            mMeshShader.drawIndexed(GL_TRIANGLES, 0, mRes.faceCount());
        }
        glDisable(GL_POLYGON_OFFSET_FILL);
    }

    if (mLayers[BoundaryWireframe]->checked()) {
        mMeshShader.bind();
        mMeshShader.setUniform("light_position", mLightPosition);
        mMeshShader.setUniform("model", model);
        mMeshShader.setUniform("view", view);
        mMeshShader.setUniform("proj", proj);
        mMeshShader.setUniform("base_color", Vector4f(Vector4f::Constant(0.f)));
        mMeshShader.setUniform("specular_color", Vector4f(Vector4f::Constant(0.f)));
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        mMeshShader.drawIndexed(GL_TRIANGLES, 0, mRes.faceCount());
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    if (mLayers[ExtractionResult]->checked()) {
        auto &shader = mExtractionResultShader;
        shader.bind();
        shader.setUniform("split", mSplit, false);
        shader.setUniform("mvp", mvp);
		shader.drawArray(GL_LINES, 0, mRes.E_rend.cols());
	}
	if (mLayers[ExtractionResult2]->checked()) {
		auto &shader = mExtractionResultShader2;
		shader.bind();
		shader.setUniform("split", mSplit, false);
		shader.setUniform("mvp", mvp);
		shader.drawArray(GL_LINES, 0, mRes.E_I_rend.cols());
	}
	if (mMorph2Box->checked()) {
		auto &shader = mEdge_color_morph2;
		shader.bind();
		shader.setUniform("split", mSplit, false);
		shader.setUniform("mvp", mvp);
		shader.drawArray(GL_LINES, 0, mRes.E_I_rend_o.cols());
	}	

	if (mShow_F_local->checked()) {
		mExtractionResultShader_F_done_local.bind();
		mExtractionResultShader_F_done_local.setUniform("light_position", mLightPosition);
		mExtractionResultShader_F_done_local.setUniform("model", model);
		mExtractionResultShader_F_done_local.setUniform("view", view);
		mExtractionResultShader_F_done_local.setUniform("proj", proj);
		mExtractionResultShader_F_done_local.setUniform("base_color", mBaseColorBoundary);
		//mExtractionResultShader_F.setUniform("specular_color", mSpecularColorBoundary);
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0, 1.0);
		mExtractionResultShader_F_done_local.drawIndexed(GL_TRIANGLES, 0, mRes.F_tag_rend.cols());
		glDisable(GL_POLYGON_OFFSET_FILL);
	}
	if (mShow_E_tagging->checked())
	{
		auto &shader = mExtractionResultShader_E_tagging;
		shader.bind();
		shader.setUniform("split", mSplit, false);
		shader.setUniform("mvp", mvp);
		shader.drawArray(GL_LINES, 0, mRes.E_tag_rend.cols());
	}
	if (mShow_E_degeneracy->checked())
	{
		auto &shader = mExtractionResultShader_E_degeneracy;
		shader.bind();
		shader.setUniform("split", mSplit, false);
		shader.setUniform("mvp", mvp);
		shader.drawArray(GL_LINES, 0, mRes.E_tag_left_rend.cols());
	}
	if (mShow_E_triangle->checked())
	{
		auto &shader = mExtractionResultShader_E_triangle;
		shader.bind();
		shader.setUniform("split", mSplit, false);
		shader.setUniform("mvp", mvp);
		shader.drawArray(GL_LINES, 0, mRes.E_tag_rend.cols());
	}

	if (mShow_F_done->checked()) {
		mExtractionResultShader_F_done.bind();
		mExtractionResultShader_F_done.setUniform("light_position", mLightPosition);
		mExtractionResultShader_F_done.setUniform("model", model);
		mExtractionResultShader_F_done.setUniform("view", view);
		mExtractionResultShader_F_done.setUniform("proj", proj);
		mExtractionResultShader_F_done.setUniform("base_color", mBaseColorBoundary);
		//mExtractionResultShader_F.setUniform("specular_color", mSpecularColorBoundary);
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0, 1.0);
		mExtractionResultShader_F_done.drawIndexed(GL_TRIANGLES, 0, mRes.F_final_rend.cols() * 3);
		glDisable(GL_POLYGON_OFFSET_FILL);
	}
	if (mShow_E_done->checked())
	{
		auto &shader = mExtractionResultShader_E_done;
		shader.bind();
		shader.setUniform("split", mSplit, false);
		shader.setUniform("mvp", mvp);
		shader.drawArray(GL_LINES, 0, mRes.E_final_rend.cols());
	}

	if (mLayers[VertexLabels]->checked()) {
		nvgBeginFrame(mNVGContext, mSize[0], mSize[1], mPixelRatio);
		nvgFontSize(mNVGContext, 14.0f);
		nvgFontFace(mNVGContext, "sans-bold");
		nvgTextAlign(mNVGContext, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
		//const MatrixXf &V = mRes.V();
		const MatrixXf &V = mRes.mV_tag;
		nvgFillColor(mNVGContext, Color(255, 200, 200, 200));

		for (uint32_t i = 0; i<V.cols(); ++i) {

			if (mVertexIdBox->value() != -1 && i != mVertexIdBox->value())
				continue;

			Vector4f pos;
			pos << V.col(i).cast<float>(), 1.0f;

			if (pos.dot(mSplit) < 0)
				continue;

			Eigen::Vector3f coord = project(Vector3f((model * pos).head<3>()), view, proj, mSize);
			if (coord.x() < -50 || coord.x() > mSize[0] + 50 || coord.y() < -50 || coord.y() > mSize[1] + 50 || coord.z() > 1)
				continue;
			Vector3f ray_origin = pos.head<3>();// +n * pos.cwiseAbs().maxCoeff() * 1e-4f;
			//if (!mRes.bvh()->rayIntersect(Ray(ray_origin, civ.head<3>() - ray_origin, 0.0f, 1.1f)))
				nvgText(mNVGContext, coord.x(), mSize[1] - coord.y(), std::to_string(i).c_str(), nullptr);

		}
		nvgEndFrame(mNVGContext);
	}

    if (mLayers[FaceLabels]->checked()) {
        nvgBeginFrame(mNVGContext, mSize[0], mSize[1], mPixelRatio);
        nvgFontSize(mNVGContext, 14.0f);
        nvgFontFace(mNVGContext, "sans-bold");
        nvgTextAlign(mNVGContext, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
        const MatrixXf &V = mRes.V();
        nvgFillColor(mNVGContext, Color(200, 200, 255, 200));

        auto const &F = mRes.F();

        for (uint32_t i=0; i<F.cols(); ++i) {
			if (mFaceIdBox->value() != -1 && i != mFaceIdBox->value())
				continue;
            Vector3f v0 = V.col(F(0, i)), v1 = V.col(F(1, i)),
                     v2 = V.col(F(2, i));
            Vector4f pos;
            pos << (1.0f / 3.0f) * (v0+v1+v2).cast<float>(), 1.0f;

            Vector3f n = (v1-v0).cross(v2-v0).normalized();
            if (pos.dot(mSplit) < 0)
                continue;

            Eigen::Vector3f coord = project(Vector3f((model * pos).head<3>()), view, proj, mSize);
            if (coord.x() < -50 || coord.x() > mSize[0] + 50 || coord.y() < -50 || coord.y() > mSize[1] + 50 || coord.z() > 1)
                continue;
            Vector3f ray_origin = pos.head<3>() + n * pos.cwiseAbs().maxCoeff() * 1e-4f;
            if (!mRes.bvh()->rayIntersect(Ray(ray_origin, civ.head<3>() - ray_origin, 0.0f, 1.1f)))
                nvgText(mNVGContext, coord.x(), mSize[1] - coord.y(), std::to_string(i).c_str(), nullptr);

        }
        nvgEndFrame(mNVGContext);
    }
	if (mLayers[Face_afterLabels]->checked()) {
		nvgBeginFrame(mNVGContext, mSize[0], mSize[1], mPixelRatio);
		nvgFontSize(mNVGContext, 14.0f);
		nvgFontFace(mNVGContext, "sans-bold");
		nvgTextAlign(mNVGContext, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
		const MatrixXf &V = mRes.V();
		nvgFillColor(mNVGContext, Color(200, 200, 255, 200));

		auto const &F = mRes.F_tag;

		for (uint32_t i = 0; i<F.size(); ++i) {
			if (mFaceIdBox->value() != -1 && i != mFaceIdBox->value())
				continue;
			Vector3f v, v0, v1, v2; v.setZero();
			for (auto vid : F[i]) v += mRes.mV_tag.col(vid);
			v /= F[i].size();
			Vector4f pos;
			pos << v, 1.0f;

			if (F[i].size() < 3) continue;
			v0 = mRes.mV_tag.col(F[i][0]);
			v1 = mRes.mV_tag.col(F[i][1]);
			v2 = mRes.mV_tag.col(F[i][2]);

			Vector3f n = (v1 - v0).cross(v2 - v0).normalized();
			if (pos.dot(mSplit) < 0)
				continue;

			Eigen::Vector3f coord = project(Vector3f((model * pos).head<3>()), view, proj, mSize);
			if (coord.x() < -50 || coord.x() > mSize[0] + 50 || coord.y() < -50 || coord.y() > mSize[1] + 50 || coord.z() > 1)
				continue;
			Vector3f ray_origin = pos.head<3>() + n * pos.cwiseAbs().maxCoeff() * 1e-4f;
			if (!mRes.bvh()->rayIntersect(Ray(ray_origin, civ.head<3>() - ray_origin, 0.0f, 1.1f)))
				nvgText(mNVGContext, coord.x(), mSize[1] - coord.y(), std::to_string(i).c_str(), nullptr);

		}
		nvgEndFrame(mNVGContext);
	}	
}

bool Viewer::keyboardEvent(int key, int scancode, int action, int modifiers) {
    if (Screen::keyboardEvent(key, scancode, action, modifiers))
        return true;
    if (action != GLFW_PRESS)
        return false;
    if (key == 'S') {
        try {
            saveState("state.bin");
            return true;
        } catch (const std::exception &ex) {
            cout << "Could not save current state: "<< ex.what() << endl;
        }
    } else if (key == 'L') {
        try {
            loadState("state.bin");
            return true;
        } catch (const std::exception &ex) {
            cout << "Could not load current state: "<< ex.what() << endl;
        }
    }


    return false;
}

void Viewer::saveState(const std::string &filename) {
    Timer<> timer;
    timer.beginStage("Writing application state to \"" + filename + "\"");
    Serializer serializer(filename, true);
    mRes.save(serializer);
    mOptimizer->save(serializer);
    serializer.set("gui", (Widget&) *this);
    serializer.set("tetShader", mTetShader);
    serializer.set("meshShader", mMeshShader);
    serializer.set("orientationFieldShaderTri", mOrientationFieldShaderTri);
    serializer.set("orientationFieldShaderTet", mOrientationFieldShaderTet);
    serializer.set("orientationSingularityShaderTri", mOrientationSingularityShaderTri);
    serializer.set("orientationSingularityShaderTet", mOrientationSingularityShaderTet);
    serializer.set("positionFieldShader", mPositionFieldShader);
    serializer.set("positionSingularityShaderTri", mPositionSingularityShaderTri);
    serializer.set("positionSingularityShaderTet", mPositionSingularityShaderTet);
    serializer.set("mSplit", mSplit);
    serializer.set("mLightPosition", mLightPosition);
    serializer.push("camera");
    serializer.set("zoom", mCamera.zoom);
    serializer.set("viewAngle", mCamera.viewAngle);
    serializer.set("dnear", mCamera.dnear);
    serializer.set("dfar", mCamera.dfar);
    serializer.set("eye", mCamera.eye);
    serializer.set("center", mCamera.center);
    serializer.set("up", mCamera.up);
    serializer.set("modelTranslation", mCamera.modelTranslation);
    serializer.set("modelZoom", mCamera.modelZoom);
    serializer.set("arcball", mCamera.arcball.state());
    serializer.pop();
    timer.endStage(memString(serializer.size()));
}

void Viewer::loadState(const std::string &filename) {
    Timer<> timer;
    timer.beginStage("Loading application state from \"" + filename + "\"");
    Serializer serializer(filename, false);
    mRes.load(serializer);
    mOptimizer->load(serializer);
    serializer.get("gui", (Widget&) *this);
    serializer.get("tetShader", mTetShader);
    serializer.get("meshShader", mMeshShader);
    serializer.get("orientationFieldShaderTet", mOrientationFieldShaderTet);
    serializer.get("orientationFieldShaderTri", mOrientationFieldShaderTri);
    serializer.get("positionFieldShader", mPositionFieldShader);
    serializer.get("orientationSingularityShaderTri", mOrientationSingularityShaderTri);
    serializer.get("orientationSingularityShaderTet", mOrientationSingularityShaderTet);
    serializer.get("positionSingularityShaderTri", mPositionSingularityShaderTri);
    serializer.get("positionSingularityShaderTet", mPositionSingularityShaderTet);
    serializer.get("mSplit", mSplit);
    serializer.get("mLightPosition", mLightPosition);
    serializer.push("camera");
    serializer.get("zoom", mCamera.zoom);
    serializer.get("viewAngle", mCamera.viewAngle);
    serializer.get("dnear", mCamera.dnear);
    serializer.get("dfar", mCamera.dfar);
    serializer.get("eye", mCamera.eye);
    serializer.get("center", mCamera.center);
    serializer.get("up", mCamera.up);
    serializer.get("modelTranslation", mCamera.modelTranslation);
    serializer.get("modelZoom", mCamera.modelZoom);
    Quaternionf arcballState;
    serializer.get("arcball", arcballState);
    mCamera.arcball.setState(arcballState);
    serializer.pop();
    timer.endStage(memString(serializer.size()));
}

void Viewer::computeCameraMatrices(Eigen::Matrix4f &model,
                                   Eigen::Matrix4f &view,
                                   Eigen::Matrix4f &proj) {
    view = lookAt(mCamera.eye, mCamera.center, mCamera.up);

    float fH = std::tan(mCamera.viewAngle / 360.0f * M_PI) * mCamera.dnear;
    float fW = fH * (float) mSize.x() / (float) mSize.y();

    proj = frustum(-fW, fW, -fH, fH, mCamera.dnear, mCamera.dfar);
    model = mCamera.arcball.matrix();

    model = scale(model, Eigen::Vector3f::Constant(mCamera.zoom * mCamera.modelZoom));
    model = translate(model, mCamera.modelTranslation);
}


