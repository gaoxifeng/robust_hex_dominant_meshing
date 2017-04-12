#include "viewer.h"
#include "im_resources.h"
#include "timer.h"
#include "bvh.h"
#include <nanogui/serializer/opengl.h>

Viewer::Viewer(std::string &filename, bool fullscreen)
    : Screen(Vector2i(1280, 960), "Robust Quad/Hex-dominant Meshes", true),
      mOptimizer(nullptr) {

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
	}
	mExampleImages.insert(mExampleImages.begin(),
		std::make_pair(nvgImageIcon(ctx, loadmesh), ""));

	/* Initialize user interface */
    Window *window = new Window(this, "");
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
				{ "obj", "Wavefront OBJ" }
			}, false);
			if (filename2 == "")
				return;
		}

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

//Config Layers
	PopupButton *openBtn3 = new PopupButton(window, "Config Layers");
	auto popup3 = openBtn3->popup();
	popup3->setLayout(new GroupLayout());
	Configlayers[Config_Layers::Alignment] = new CheckBox(popup3, "Boundary alignment");
	Configlayers[Config_Layers::Extrinsic] = new CheckBox(popup3, "Extrinsic smoothing");
	Configlayers[Config_Layers::Randomization] = new CheckBox(popup3, "Randomization");
	Configlayers[Config_Layers::Hierarchy] = new CheckBox(popup3, "Hierarchy");
	int ctr = 0;
	for (auto l : Configlayers) {
		l->setChecked(true);
		l->setId("configlayer." + std::to_string(ctr++));
	}
//Render Layers
    PopupButton *openBtn = new PopupButton(window, "Render Layers");
    auto popup = openBtn->popup();
    popup->setLayout(new GroupLayout());

    mLayers[Layers::Tetrahedra]               = new CheckBox(popup, "Tetrahedra");
    mLayers[Layers::OrientationField]         = new CheckBox(popup, "Orientation field");
    mLayers[Layers::OrientationSingularities] = new CheckBox(popup, "Orientation singularities");
    mLayers[Layers::PositionField]            = new CheckBox(popup, "Position field");
    mLayers[Layers::PositionSingularities]    = new CheckBox(popup, "Position singularities");
    mLayers[Layers::Boundary]                 = new CheckBox(popup, "Boundary");
    mLayers[Layers::BoundaryWireframe]        = new CheckBox(popup, "Boundary wireframe");

    ctr = 0;
    for (auto l : mLayers) {
        l->setChecked(false);
        l->setId("layer." + std::to_string(ctr++));
    }
//Parameters
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

//build structure	
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
	
//Rosy
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
//Posy
	mSolvePositionBtn = new Button(window, "Posy", ENTYPO_ICON_FLASH);
    mSolvePositionBtn->setBackgroundColor(Color(0, 0, 255, 25));
    mSolvePositionBtn->setFlags(Button::Flags::ToggleButton);
    mSolvePositionBtn->setChangeCallback([&](bool value) {

        mOptimizer->setOptimizePositions(value);
        mOptimizer->notify();

        mLayers[Layers::OrientationField]->setChecked(false);
        mLayers[Layers::OrientationSingularities]->setChecked(false);
        mLayers[Layers::PositionField]->setChecked(true);
        mLayers[Layers::PositionSingularities]->setChecked(true);
        if (value == false)
            updatePositionSingularities();
    });

	PopupButton *MorphingBtn = new PopupButton(window, "Morphing");
	Popup *morphPopup = MorphingBtn->popup();
	morphPopup->setAnchorHeight(61);

	morphPopup->setLayout(new GroupLayout());

	mEdgeTagging_done = false;
	mEdgeTaggingBtn = new Button(morphPopup, "Coloring", ENTYPO_ICON_FLASH);
	mEdgeTaggingBtn->setBackgroundColor(Color(0, 0, 255, 25));
	mEdgeTaggingBtn->setFlags(Button::Flags::ToggleButton);
	mEdgeTaggingBtn->setChangeCallback([&](bool value) {
		if (!mRes.tetMesh())
			mRes.init_edge_tagging2D();
		else
		{
			mRes.init_edge_tagging3D();
		}
		mEdgeTagging_done = true;
		auto const &R = mRes.E_rend;
		mExtractionResultShader.bind();
		mExtractionResultShader.uploadAttrib("position", MatrixXf(R.block(0, 0, 3, R.cols())));
		mExtractionResultShader.uploadAttrib("color", MatrixXf(R.block(3, 0, 3, R.cols())));
	});
	Slider *slider2 = new Slider(morphPopup);
	slider2->setValue(0.0);
	auto cb = [&](Float value) {
		mRes.E_I_rend = (1 - value) * mRes.E_rend + value*mRes.E_O_rend;
	};
	cb(0.0f);
	slider2->setCallback(cb);
	slider2->setId("slider2");
//Extraction
    mExtractBtn = new Button(window, "Extract", ENTYPO_ICON_FLASH);
    mExtractBtn->setBackgroundColor(Color(0, 255, 0, 25));
    mExtractBtn->setCallback([&]() {
		if (!mRes.tetMesh()) {

			mRes.re_color = true;
			mRes.doublets = true;
			mRes.splitting = true;
			mRes.triangles = true;
			mRes.decomposes = true;

			mRes.meshExtraction2D();
		}
		else{
			mRes.re_color = true;
			mRes.splitting = true;
			mRes.doublets = false;
			mRes.triangles = false;
			mRes.decomposes = false;
			mRes.meshExtraction3D();
		}
		mLayers[PositionSingularities]->setChecked(false);
		mLayers[Layers::PositionField]->setChecked(false);
		mLayers[Layers::Boundary]->setChecked(false);
		mLayers[OrientationSingularities]->setChecked(false);
		mLayers[Layers::OrientationField]->setChecked(false);

		mExtractionResultShader_F_done.bind();
		mExtractionResultShader_F_done.uploadAttrib("position", mRes.mV_final_rend);
		mExtractionResultShader_F_done.uploadIndices(mRes.F_final_rend);
		mShow_F_done->setChecked(true);

		auto const &R4 = mRes.E_final_rend;
		mExtractionResultShader_E_done.bind();
		mExtractionResultShader_E_done.uploadAttrib("position", MatrixXf(R4.block(0, 0, 3, R4.cols())));
		mExtractionResultShader_E_done.uploadAttrib("color", MatrixXf(R4.block(3, 0, 3, R4.cols())));
		mShow_E_done->setChecked(true);
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
//output
	PopupButton *ConstraintsBtn = new PopupButton(window, "Output");
	ConstraintsBtn->setIcon(ENTYPO_ICON_ROCKET);
	ConstraintsBtn->setBackgroundColor(Color(100, 0, 0, 25));
	Popup *ConstraintsPopup = ConstraintsBtn->popup();
	ConstraintsPopup->setAnchorHeight(61);

	ConstraintsPopup->setLayout(new GroupLayout());

	mShow_F_done = new CheckBox(ConstraintsPopup, "Face");
	mShow_F_done->setId("showdoneF");
	mShow_F_done->setChecked(false);
	mShow_E_done = new CheckBox(ConstraintsPopup, "Edge");
	mShow_E_done->setId("showdoneE");
	mShow_E_done->setChecked(false);


	mOutputBtn = new Button(ConstraintsPopup, "Output", ENTYPO_ICON_FLASH);
	mOutputBtn->setBackgroundColor(Color(0, 255, 0, 25));
	mOutputBtn->setCallback([&]() {
		char patho[300];
		if (!mRes.tetMesh()) {
			sprintf(patho, "%s%s", filename.c_str(), "_surout.obj");
			write_surface_mesh_OBJ(mRes.mV_tag, mRes.F_tag, patho);
		}
		else {
			sprintf(patho, "%s%s", filename.c_str(), ".HYBRID");
			write_volume_mesh_HYBRID(mRes.mV_tag, mRes.F_tag, mRes.P_tag, mRes.Hex_flag, mRes.PF_flag, patho);
		}
	});
//layout
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

	mOptimizer->setAlignment(Configlayers[Config_Layers::Alignment]->checked());
	mOptimizer->setRandomization(Configlayers[Config_Layers::Randomization]->checked());
	mOptimizer->setHierarchy(Configlayers[Config_Layers::Hierarchy]->checked());

	mRes.setScale(mScaleBox->value());

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

 //   if (mLayers[ExtractionResult]->checked()) {
 //       auto &shader = mExtractionResultShader;
 //       shader.bind();
 //       shader.setUniform("split", mSplit, false);
 //       shader.setUniform("mvp", mvp);
	//	shader.drawArray(GL_LINES, 0, mRes.E_rend.cols());
	//}
	//if (mLayers[ExtractionResult2]->checked()) {
	//	auto &shader = mExtractionResultShader2;
	//	shader.bind();
	//	shader.setUniform("split", mSplit, false);
	//	shader.setUniform("mvp", mvp);
	//	shader.drawArray(GL_LINES, 0, mRes.E_I_rend.cols());
	//}

	if (mShow_F_done->checked()) {
		mExtractionResultShader_F_done.bind();
		mExtractionResultShader_F_done.setUniform("light_position", mLightPosition);
		mExtractionResultShader_F_done.setUniform("model", model);
		mExtractionResultShader_F_done.setUniform("view", view);
		mExtractionResultShader_F_done.setUniform("proj", proj);
		mExtractionResultShader_F_done.setUniform("base_color", mBaseColorBoundary);
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


