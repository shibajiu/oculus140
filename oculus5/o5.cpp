#include "o5.h"

int main(){
	if (init() == -1)
		goto done;

done:
	ovr_Destroy(session);
	ovr_Shutdown();
	return 0;
}

int init(){
	if (OVR_SUCCESS(ovr_Initialize(nullptr)))
		printf_s("ovrLIB initialization success");
	else
		return -1;
	
	if (OVR_FAILURE(ovr_Create(&session, &luid)))		
		return -1;

	ovrTrackingState ts = ovr_GetTrackingState(session, ovr_GetTimeInSeconds(), ovrTrue);
	if (ts.StatusFlags & (ovrStatus_OrientationTracked | ovrStatus_PositionTracked)){
		ovrPosef pose = ts.HeadPose.ThePose;
	}
	hmdDesc = ovr_GetHmdDesc(session);
	windowSize = { hmdDesc.Resolution.w *0.5, hmdDesc.Resolution.h *0.5 };

#pragma region glfw

	//create a glfw window
	glfwSetErrorCallback(error_callback);
	if (!glfwInit())
		exit(EXIT_FAILURE);
	else
		printf("glfw inited\n");
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
	if (!(window = glfwCreateWindow(windowSize.w, windowSize.h, "Oculus", NULL, NULL))){
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	else
		printf("glfw created window\n");
	glfwMakeContextCurrent(window);
	glewInit();//donnot forget!!
	// Turn off vsync to let the compositor do its magic
	glfwSwapInterval(0);

#pragma endregion initialize glfw
	
	for (int eye = 0; eye < 2; ++eye){
		ovrSizei idealTextureSize = ovr_GetFovTextureSize(session, ovrEyeType(eye), hmdDesc.DefaultEyeFov[eye], 1);
		eyeRenderTexture[eye] = new TextureBuffer(session, true, true, idealTextureSize, 1, NULL, 1);
		eyeDepthBuffer[eye] = new DepthBuffer(eyeRenderTexture[eye]->GetSize(), 0);

		if (!eyeRenderTexture[eye]->TextureChain){			
			fprintf(stderr, "Failed to create texture.");
			return -1;
		}
	}

	memset(&mirrorDesc, 0, sizeof(mirrorDesc));
	mirrorDesc.Width = windowSize.w;
	mirrorDesc.Height = windowSize.h;
	mirrorDesc.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;
	// Create mirror texture and an FBO used to copy mirror texture to back buffer
	ovr_CreateMirrorTextureGL(session, &mirrorDesc, &mirrorTexture);
	

	// Configure the mirror read buffer
	GLuint texId;
	ovr_GetMirrorTextureBufferGL(session, mirrorTexture, &texId);

	glGenFramebuffers(1, &mirrorFBO);
	glBindFramebuffer(GL_READ_FRAMEBUFFER, mirrorFBO);
	glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texId, 0);
	glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
	glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

	// FloorLevel will give tracking poses where the floor height is 0
	ovr_SetTrackingOriginType(session, ovrTrackingOrigin_FloorLevel);

	ld.Header.Type = ovrLayerType_EyeFov;
	ld.Header.Flags = ovrLayerFlag_TextureOriginAtBottomLeft;   // Because OpenGL.
	for (int eye = 0; eye < 2; ++eye){
		ld.ColorTexture[eye] = eyeRenderTexture[eye]->TextureChain;
		ld.Viewport[eye] = Recti(eyeRenderTexture[eye]->GetSize());
		ld.Fov[eye] = hmdDesc.DefaultEyeFov[eye];		
	}

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_NORMALIZE);
	float V[] = { 1, 1, 1, 1 };
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, V);
	glEnable(GL_COLOR_MATERIAL);

	glClearColor(1, 1, 1, 1);
	chess_tex = gen_chess_tex(1.0, 0.7, 0.4, 0.4, 0.7, 1.0);

}

void main_loop(){
	// Call ovr_GetRenderDesc each frame to get the ovrEyeRenderDesc, as the returned values (e.g. HmdToEyeOffset) may change at runtime.
	ovrEyeRenderDesc eyeRenderDesc[2];
	eyeRenderDesc[0] = ovr_GetRenderDesc(session, ovrEye_Left, hmdDesc.DefaultEyeFov[0]);
	eyeRenderDesc[1] = ovr_GetRenderDesc(session, ovrEye_Right, hmdDesc.DefaultEyeFov[1]);

	// Get eye poses, feeding in correct IPD offset
	ovrPosef                  EyeRenderPose[2];
	ovrVector3f               HmdToEyeOffset[2] = { eyeRenderDesc[0].HmdToEyeOffset,
		eyeRenderDesc[1].HmdToEyeOffset };
	float rot_mat[16];
	double sensorSampleTime;    // sensorSampleTime is fed into the layer later
	ld.SensorSampleTime = sensorSampleTime;
	ovr_GetEyePoses(session, frameIndex, ovrTrue, HmdToEyeOffset, EyeRenderPose, &sensorSampleTime);

	if (isVisible){
		for (int eye = 0; eye < 2; ++eye){
			// Switch to eye render target
			eyeRenderTexture[eye]->SetAndClearRenderSurface(eyeDepthBuffer[eye]);
			
			Matrix4f proj = ovrMatrix4f_Projection(hmdDesc.DefaultEyeFov[eye], 0.2f, 1000.0f, ovrProjection_None);

			// Render world
			glMatrixMode(GL_PROJECTION);
			glLoadTransposeMatrixf(proj.M[0]);

			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			glTranslatef(HmdToEyeOffset[eye].x,HmdToEyeOffset[eye].y,HmdToEyeOffset[eye].z);
			quat_to_matrix(&ld.RenderPose[eye].Orientation.x, rot_mat);
			glMultMatrixf(rot_mat);
			/* translate the view matrix with the positional tracking */
			glTranslatef(-EyeRenderPose[eye].Position.x, -EyeRenderPose[eye].Position.y, -EyeRenderPose[eye].Position.z);
			/* move the camera to the eye level of the user */
			glTranslatef(0, -ovr_GetFloat(session, OVR_KEY_EYE_HEIGHT, 1.65), 0);

			draw_scene();

			// Avoids an error when calling SetAndClearRenderSurface during next iteration.
			// Without this, during the next while loop iteration SetAndClearRenderSurface
			// would bind a framebuffer with an invalid COLOR_ATTACHMENT0 because the texture ID
			// associated with COLOR_ATTACHMENT0 had been unlocked by calling wglDXUnlockObjectsNV.
			eyeRenderTexture[eye]->UnsetRenderSurface();

			// Commit changes to the textures so they get picked up frame
			eyeRenderTexture[eye]->Commit();
		}
	}

	// Do distortion rendering, Present and flush/sync
	for (int eye = 0; eye < 2; ++eye)
		ld.RenderPose[eye] = EyeRenderPose[eye];
	
	ovrLayerHeader* layers = &ld.Header;
	ovrResult result = ovr_SubmitFrame(session, frameIndex, nullptr, &layers, 1);
	// exit the rendering loop if submit returns an error, will retry on ovrError_DisplayLost
	if (!OVR_SUCCESS(result))
		return;

	isVisible = (result == ovrSuccess);

	ovrSessionStatus sessionStatus;
	ovr_GetSessionStatus(session, &sessionStatus);
	if (sessionStatus.ShouldQuit)
		return;
	if (sessionStatus.ShouldRecenter)
		ovr_RecenterTrackingOrigin(session);

	// Blit mirror texture to back buffer
	glBindFramebuffer(GL_READ_FRAMEBUFFER, mirrorFBO);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
	GLint w = windowSize.w;
	GLint h = windowSize.h;
	glBlitFramebuffer(0, h, w, 0,
		0, 0, w, h,
		GL_COLOR_BUFFER_BIT, GL_NEAREST);
	glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

	glfwSwapBuffers(window);

	frameIndex++;	
}

void quat_to_matrix(const float *quat, float *mat){
	mat[0] = 1.0 - 2.0 * quat[1] * quat[1] - 2.0 * quat[2] * quat[2];
	mat[4] = 2.0 * quat[0] * quat[1] + 2.0 * quat[3] * quat[2];
	mat[8] = 2.0 * quat[2] * quat[0] - 2.0 * quat[3] * quat[1];
	mat[12] = 0.0f;

	mat[1] = 2.0 * quat[0] * quat[1] - 2.0 * quat[3] * quat[2];
	mat[5] = 1.0 - 2.0 * quat[0] * quat[0] - 2.0 * quat[2] * quat[2];
	mat[9] = 2.0 * quat[1] * quat[2] + 2.0 * quat[3] * quat[0];
	mat[13] = 0.0f;

	mat[2] = 2.0 * quat[2] * quat[0] + 2.0 * quat[3] * quat[1];
	mat[6] = 2.0 * quat[1] * quat[2] - 2.0 * quat[3] * quat[0];
	mat[10] = 1.0 - 2.0 * quat[0] * quat[0] - 2.0 * quat[1] * quat[1];
	mat[14] = 0.0f;

	mat[3] = mat[7] = mat[11] = 0.0f;
	mat[15] = 1.0f;
}

void draw_scene(void){
	int i;
	float grey[] = { 0.8, 0.8, 0.8, 1 };
	float col[] = { 0, 0, 0, 1 };
	float lpos[][4] = {
		{ -8, 2, 10, 1 },
		{ 0, 15, 0, 1 }
	};
	float lcol[][4] = {
		{ 0.8, 0.8, 0.8, 1 },
		{ 0.4, 0.3, 0.3, 1 }
	};

	for (i = 0; i<2; i++) {
		glLightfv(GL_LIGHT0 + i, GL_POSITION, lpos[i]);
		glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, lcol[i]);
	}

	glMatrixMode(GL_MODELVIEW);

	glPushMatrix();
	glTranslatef(0, 10, 0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, grey);
	glBindTexture(GL_TEXTURE_2D, chess_tex);
	glEnable(GL_TEXTURE_2D);
	draw_box(30, 20, 30, -1.0);
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();

	for (i = 0; i<4; i++) {
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, grey);
		glPushMatrix();
		glTranslatef(i & 1 ? 5 : -5, 1, i & 2 ? -5 : 5);
		draw_box(0.5, 2, 0.5, 1.0);
		glPopMatrix();

		col[0] = i & 1 ? 1.0 : 0.3;
		col[1] = i == 0 ? 1.0 : 0.3;
		col[2] = i & 2 ? 1.0 : 0.3;
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, col);

		glPushMatrix();
		if (i & 1) {
			glTranslatef(0, 0.25, i & 2 ? 2 : -2);
		}
		else {
			glTranslatef(i & 2 ? 2 : -2, 0.25, 0);
		}
		draw_box(0.5, 0.5, 0.5, 1.0);
		glPopMatrix();
	}

	col[0] = 1;
	col[1] = 1;
	col[2] = 0.4;
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, col);
	draw_box(0.05, 1.2, 6, 1.0);
	draw_box(6, 1.2, 0.05, 1.0);
}

void draw_box(float xsz, float ysz, float zsz, float norm_sign){
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glScalef(xsz * 0.5, ysz * 0.5, zsz * 0.5);

	if (norm_sign < 0.0) {
		glFrontFace(GL_CW);
	}

	glBegin(GL_QUADS);
	glNormal3f(0, 0, 1 * norm_sign);
	glTexCoord2f(0, 0); glVertex3f(-1, -1, 1);
	glTexCoord2f(1, 0); glVertex3f(1, -1, 1);
	glTexCoord2f(1, 1); glVertex3f(1, 1, 1);
	glTexCoord2f(0, 1); glVertex3f(-1, 1, 1);
	glNormal3f(1 * norm_sign, 0, 0);
	glTexCoord2f(0, 0); glVertex3f(1, -1, 1);
	glTexCoord2f(1, 0); glVertex3f(1, -1, -1);
	glTexCoord2f(1, 1); glVertex3f(1, 1, -1);
	glTexCoord2f(0, 1); glVertex3f(1, 1, 1);
	glNormal3f(0, 0, -1 * norm_sign);
	glTexCoord2f(0, 0); glVertex3f(1, -1, -1);
	glTexCoord2f(1, 0); glVertex3f(-1, -1, -1);
	glTexCoord2f(1, 1); glVertex3f(-1, 1, -1);
	glTexCoord2f(0, 1); glVertex3f(1, 1, -1);
	glNormal3f(-1 * norm_sign, 0, 0);
	glTexCoord2f(0, 0); glVertex3f(-1, -1, -1);
	glTexCoord2f(1, 0); glVertex3f(-1, -1, 1);
	glTexCoord2f(1, 1); glVertex3f(-1, 1, 1);
	glTexCoord2f(0, 1); glVertex3f(-1, 1, -1);
	glEnd();
	glBegin(GL_TRIANGLE_FAN);
	glNormal3f(0, 1 * norm_sign, 0);
	glTexCoord2f(0.5, 0.5); glVertex3f(0, 1, 0);
	glTexCoord2f(0, 0); glVertex3f(-1, 1, 1);
	glTexCoord2f(1, 0); glVertex3f(1, 1, 1);
	glTexCoord2f(1, 1); glVertex3f(1, 1, -1);
	glTexCoord2f(0, 1); glVertex3f(-1, 1, -1);
	glTexCoord2f(0, 0); glVertex3f(-1, 1, 1);
	glEnd();
	glBegin(GL_TRIANGLE_FAN);
	glNormal3f(0, -1 * norm_sign, 0);
	glTexCoord2f(0.5, 0.5); glVertex3f(0, -1, 0);
	glTexCoord2f(0, 0); glVertex3f(-1, -1, -1);
	glTexCoord2f(1, 0); glVertex3f(1, -1, -1);
	glTexCoord2f(1, 1); glVertex3f(1, -1, 1);
	glTexCoord2f(0, 1); glVertex3f(-1, -1, 1);
	glTexCoord2f(0, 0); glVertex3f(-1, -1, -1);
	glEnd();

	glFrontFace(GL_CCW);
	glPopMatrix();
}

unsigned int gen_chess_tex(float r0, float g0, float b0, float r1, float g1, float b1){
	int i, j;
	unsigned int tex;
	unsigned char img[8 * 8 * 3];
	unsigned char *pix = img;

	for (i = 0; i<8; i++) {
		for (j = 0; j<8; j++) {
			int black = (i & 1) == (j & 1);
			pix[0] = (black ? r0 : r1) * 255;
			pix[1] = (black ? g0 : g1) * 255;
			pix[2] = (black ? b0 : b1) * 255;
			pix += 3;
		}
	}

	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 8, 8, 0, GL_RGB, GL_UNSIGNED_BYTE, img);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB8_ALPHA8, 8, 8, 0, GL_RGBA, GL_UNSIGNED_BYTE, img);
	printf("generated texture\n");
	return tex;
}

static void error_callback(int error, const char* description){
	fputs(description, stderr);
}