#include <GL/freeglut.h>
#include <GL/glu.h>
#include <math.h>
#include <vector>
#include <sys/timeb.h>
#include <cstdlib>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdint>
#include <sstream>
#include <algorithm>
#include <unordered_map>
#define scalefactor 0.05
#define VOXEL_SIZE 0.05f
#define FORCE_WHITE true


using namespace std;




struct XYZ_RGB_t {
	unsigned char R, G, B;
	float X = 0, Y = 0, Z = 0;
};
vector<XYZ_RGB_t> volume_t;
int old_rot_x = 0;
int old_rot_y = 0;

int rot_x = 0;
int rot_y = 0;

int record_x = 0;
int record_y = 0;

float x_shift = 0;
float y_shift = 0;
float z_shift = 0;

long long getSystemTime() {
	struct timeb t;
	ftime(&t);
	return 1000 * t.time + t.millitm;
}

void Display()
{
	glEnable(GL_DEPTH_TEST);

	glClearColor(0, 0, 0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPolygonMode(GL_BACK, GL_LINE);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(0, 0, 20, 0, 0, 0, 0, 1, 0);

	glTranslatef(x_shift, y_shift, z_shift);
	glRotatef((float)rot_y + (float)record_y, 1.0, 0.0, 0.0);
	glRotatef((float)rot_x + (float)record_x, 0.0, 1.0, 0.0);

	float scale = scalefactor;
	float x, y, z;//
	float R, G, B;//color 
	for (int n = 0; n < volume_t.size(); n++) {	//data num 
		glBegin(GL_QUADS);
		R = volume_t[n].R / 255.0, G = volume_t[n].G / 255.0, B = volume_t[n].B / 255.0;
		//printf("%.2f %.2f %.2f \n", R,G,B);
		glColor3f(R, G, B);//normalize
		x = volume_t[n].X;
		y = -volume_t[n].Y;
		z = -volume_t[n].Z;

		// Voxel //方塊有6個面，每個面有四個點
		glVertex3f(x + (-0.5 * scale), y + (0.5 * scale), z + (0.5 * scale));/*點1*/ glVertex3f(x + (-0.5 * scale), y + (-0.5 * scale), z + (0.5 * scale));/*點2*/  glVertex3f(x + (0.5 * scale), y + (-0.5 * scale), z + (0.5 * scale));/*點3*/
		glVertex3f(x + (0.5 * scale), y + (0.5 * scale), z + (0.5 * scale));/*點4*/

		glVertex3f(x + (0.5 * scale), y + (0.5 * scale), z + (-0.5 * scale));  glVertex3f(x + (0.5 * scale), y + (-0.5 * scale), z + (-0.5 * scale));  glVertex3f(x + (-0.5 * scale), y + (-0.5 * scale), z + (-0.5 * scale));
		glVertex3f(x + (-0.5 * scale), y + (0.5 * scale), z + (-0.5 * scale));

		glVertex3f(x + (0.5 * scale), y + (0.5 * scale), z + (0.5 * scale)); glVertex3f(x + (0.5 * scale), y + (-0.5 * scale), z + (0.5 * scale)); glVertex3f(x + (0.5 * scale), y + (-0.5 * scale), z + (-0.5 * scale));
		glVertex3f(x + (0.5 * scale), y + (0.5 * scale), z + (-0.5 * scale));

		glVertex3f(x + (-0.5 * scale), y + (0.5 * scale), z + (-0.5 * scale)); glVertex3f(x + (-0.5 * scale), y + (-0.5 * scale), z + (-0.5 * scale)); glVertex3f(x + (-0.5 * scale), y + (-0.5 * scale), z + (0.5 * scale));
		glVertex3f(x + (-0.5 * scale), y + (0.5 * scale), z + (0.5 * scale));

		glVertex3f(x + (-0.5 * scale), y + (0.5 * scale), z + (-0.5 * scale)); glVertex3f(x + (-0.5 * scale), y + (0.5 * scale), z + (0.5 * scale)); glVertex3f(x + (0.5 * scale), y + (0.5 * scale), z + (0.5 * scale));
		glVertex3f(x + (0.5 * scale), y + (0.5 * scale), z + (-0.5 * scale));

		glVertex3f(x + (-0.5 * scale), y + (-0.5 * scale), z + (0.5 * scale)); glVertex3f(x + (-0.5 * scale), y + (-0.5 * scale), z + (-0.5 * scale)); glVertex3f(x + (0.5 * scale), y + (-0.5 * scale), z + (-0.5 * scale));
		glVertex3f(x + (0.5 * scale), y + (-0.5 * scale), z + (0.5 * scale));

		glEnd();



	}

	glutSwapBuffers();
}


void Keyboard(unsigned char key, int x, int y)
{
	printf("你所按按鍵的碼是%x\t此時視窗內的滑鼠座標是(%d,%d)\n", key, x, y);
	switch (key)
	{
	case 'w':
		y_shift += 1.0f;
		break;
	case 's':
		y_shift -= 1.0f;
		break;
	case 'd':
		x_shift += 1.0f;
		break;
	case 'a':
		x_shift -= 1.0f;
		break;
	case 'q':
		z_shift += 1.0f;
		break;
	case 'e':
		z_shift -= 1.0f;
		break;
	case 'z':
		break;
	case 27:

		exit(0);
		break;
	}
	glutPostRedisplay();
}

void Mouse(int button, int state, int x, int y)
{
	if (state) {
		record_x += x - old_rot_x;
		record_y += y - old_rot_y;

		rot_x = 0;
		rot_y = 0;
	}
	else {
		old_rot_x = x;
		old_rot_y = y;
	}
}

void MotionMouse(int x, int y)
{
	rot_x = x - old_rot_x;
	rot_y = y - old_rot_y;
	glutPostRedisplay();
}

void WindowSize(int w, int h)
{
	printf("目前視窗大小為%dX%d\n", w, h);
	glViewport(0, 0, w, h);            //當視窗長寬改變時，畫面也跟著變
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float rate = (float)w / (float)h;
	gluPerspective(45, rate, 1.0, 74000.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

struct Point2i {
	int x, y;
};



int main(int argc, char* argv[])
{
	if (argc < 2) {
		cerr << "用法: " << argv[0] << " <點雲檔案路徑>" << endl;
		cerr << "例如: " << argv[0] << " output_points_int_LR.txt" << endl;
		return 1;
	}
	string input_path = argv[1];



	// --- 讀取原始點雲 ---
	vector<XYZ_RGB_t> raw_points;
	XYZ_RGB_t data;
	ifstream infile2(input_path);
	if (!infile2.is_open()) {
		cerr << "[Error] 無法開啟檔案: " << input_path << endl;
		return 1;
	}
	string line;
	bool has_rgb = false;
	bool has_gray = false;
	bool format_detected = false;
	while (getline(infile2, line)) {
		if (line.empty()) continue;
		replace(line.begin(), line.end(), ',', ' ');
		istringstream iss(line);
		float v[6];
		int count = 0;
		while (count < 6 && iss >> v[count]) count++;
		if (count < 3) continue;
		if (!format_detected) {
			has_rgb = (count >= 6);
			has_gray = (count == 4);
			format_detected = true;
			if (has_rgb)
				cout << "偵測到格式: XYZ + RGB" << endl;
			else if (has_gray)
				cout << "偵測到格式: XYZ + 灰階" << endl;
			else
				cout << "偵測到格式: 僅 XYZ，顏色預設灰色" << endl;
		}
		data.X = v[0]; data.Y = v[1]; data.Z = v[2];
		if (has_rgb) {
			data.R = (unsigned char)v[3]; data.G = (unsigned char)v[4]; data.B = (unsigned char)v[5];
		} else if (has_gray) {
			unsigned char g = FORCE_WHITE ? 255 : (unsigned char)v[3];
			data.R = g; data.G = g; data.B = g;
		} else {
			data.R = 200; data.G = 200; data.B = 200;
		}
		raw_points.push_back(data);
	}
	infile2.close();
	cout << "讀取完成，共 " << raw_points.size() << " 個原始點。" << endl;

	// --- Voxel Downsampling ---
	cout << "正在進行 Voxel 降採樣 (VOXEL_SIZE = " << VOXEL_SIZE << ")..." << endl;
	struct VoxelKey {
		int ix, iy, iz;
		bool operator==(const VoxelKey& o) const {
			return ix == o.ix && iy == o.iy && iz == o.iz;
		}
	};
	struct VoxelKeyHash {
		size_t operator()(const VoxelKey& k) const {
			size_t h = 0;
			h ^= hash<int>()(k.ix) + 0x9e3779b9 + (h << 6) + (h >> 2);
			h ^= hash<int>()(k.iy) + 0x9e3779b9 + (h << 6) + (h >> 2);
			h ^= hash<int>()(k.iz) + 0x9e3779b9 + (h << 6) + (h >> 2);
			return h;
		}
	};
	struct VoxelAccum {
		double sx=0, sy=0, sz=0, sr=0, sg=0, sb=0;
		int count=0;
	};
	unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash> voxel_map;
	for (auto& pt : raw_points) {
		VoxelKey key;
		key.ix = (int)floor(pt.X / VOXEL_SIZE);
		key.iy = (int)floor(pt.Y / VOXEL_SIZE);
		key.iz = (int)floor(pt.Z / VOXEL_SIZE);
		auto& acc = voxel_map[key];
		acc.sx += pt.X; acc.sy += pt.Y; acc.sz += pt.Z;
		acc.sr += pt.R; acc.sg += pt.G; acc.sb += pt.B;
		acc.count++;
	}
	for (auto& kv : voxel_map) {
		auto& acc = kv.second;
		XYZ_RGB_t p;
		p.X = (float)(acc.sx / acc.count);
		p.Y = (float)(acc.sy / acc.count);
		p.Z = (float)(acc.sz / acc.count);
		p.R = (unsigned char)(acc.sr / acc.count);
		p.G = (unsigned char)(acc.sg / acc.count);
		p.B = (unsigned char)(acc.sb / acc.count);
		volume_t.push_back(p);
	}
	cout << "降採樣完成，剩下 " << volume_t.size() << " 個 Voxel。" << endl;



	






	glutInit(&argc, argv);//
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);//single is 2D

	glutInitWindowPosition(250, 50);
	glutInitWindowSize(800, 800);//size window
	glutCreateWindow("OpenGL cube");//
	glutReshapeFunc(WindowSize);
	glutKeyboardFunc(Keyboard);
	glutMouseFunc(Mouse);
	glutMotionFunc(MotionMouse);

	glutDisplayFunc(Display);
	glutMainLoop();
	return 0;
}


//#include <GL/freeglut.h> // 請確保已安裝 FreeGLUT
//#include <vector>
//#include <iostream>
//#include <fstream>
//#include <string>
//
//using namespace std;
//
//// --------------------------------------------------------
//// 參數設定
//// --------------------------------------------------------
//// 如果點雲太稀疏或太大，可以調整這個縮放因子
//#define SCALE_FACTOR 1.0
//// 每個 Voxel (立方體) 的大小
//#define VOXEL_SIZE 10.0f  
//
//// 輸入檔案路徑 (請修改為您的檔案路徑)
//const string INPUT_TXT_PATH = "D:/code/2D_to_3D_coordinates/2D_to_3D_coordinates/output_points_int.txt";
//
//// --------------------------------------------------------
//// 資料結構
//// --------------------------------------------------------
//struct VoxelPoint {
//    int X, Y, Z;        // 3D 座標 (mm)
//    unsigned char R, G, B; // 顏色 (0-255)
//};
//
//// 全域變數：儲存所有點雲資料
//vector<VoxelPoint> voxelData;
//
//// 視角控制變數
//int old_rot_x = 0;
//int old_rot_y = 0;
//int rot_x = 0;
//int rot_y = 0;
//int record_x = 0;
//int record_y = 0;
//
//float x_shift = 0;
//float y_shift = 0;
//float z_shift = 0; // 負責縮放/前後移動
//
//// --------------------------------------------------------
//// 繪圖函式 (Display)
//// --------------------------------------------------------
//void Display()
//{
//    // 開啟深度測試，確保前後遮擋關係正確
//    glEnable(GL_DEPTH_TEST);
//
//    // 清除顏色緩衝與深度緩衝
//    glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // 背景設為深灰色，比較好看清楚點雲
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//    glMatrixMode(GL_MODELVIEW);
//    glLoadIdentity();
//
//    // 設定攝影機位置 (眼睛位置, 看向哪裡, 頭頂朝向)
//    // 預設將相機拉遠一點 (Z=-2000)，以免一開始就在物體內部
//    gluLookAt(0, 0, 0, 0, 0, 1000, 0, 1, 0);
//
//    // 應用使用者的移動與旋轉
//    glTranslatef(x_shift, y_shift, z_shift); // 平移
//    glRotatef((float)rot_y + (float)record_y, 1.0, 0.0, 0.0); // 繞 X 軸旋轉
//    glRotatef((float)rot_x + (float)record_x, 0.0, 1.0, 0.0); // 繞 Y 軸旋轉
//
//    // 開始繪製 Voxel
//    float s = VOXEL_SIZE * SCALE_FACTOR / 2.0f; // 半徑
//    //float s = 0.5f * SCALE_FACTOR;
//  // 為了效能，使用 GL_QUADS 繪製所有方塊
//    glBegin(GL_QUADS);
//
//    for (const auto& p : voxelData) {
//        // 設定顏色
//        glColor3ub(p.R, p.G, p.B);
//
//        // 座標轉換
//        // 注意：OpenGL 的 Y 軸通常朝上，而影像座標 Y 朝下
//        // 這裡加負號 (-p.Y) 是為了讓畫面轉正
//        float x = -p.X * SCALE_FACTOR;
//        float y = -p.Y * SCALE_FACTOR;
//        float z = p.Z * SCALE_FACTOR;
//
//        // 繪製立方體的六個面
//        // 前面
//        glVertex3f(x - s, y + s, z + s); glVertex3f(x - s, y - s, z + s);
//        glVertex3f(x + s, y - s, z + s); glVertex3f(x + s, y + s, z + s);
//        // 後面
//        glVertex3f(x + s, y + s, z - s); glVertex3f(x + s, y - s, z - s);
//        glVertex3f(x - s, y - s, z - s); glVertex3f(x - s, y + s, z - s);
//        // 左面
//        glVertex3f(x - s, y + s, z - s); glVertex3f(x - s, y - s, z - s);
//        glVertex3f(x - s, y - s, z + s); glVertex3f(x - s, y + s, z + s);
//        // 右面
//        glVertex3f(x + s, y + s, z + s); glVertex3f(x + s, y - s, z + s);
//        glVertex3f(x + s, y - s, z - s); glVertex3f(x + s, y + s, z - s);
//        // 上面
//        glVertex3f(x - s, y + s, z - s); glVertex3f(x - s, y + s, z + s);
//        glVertex3f(x + s, y + s, z + s); glVertex3f(x + s, y + s, z - s);
//        // 下面
//        glVertex3f(x - s, y - s, z + s); glVertex3f(x - s, y - s, z - s);
//        glVertex3f(x + s, y - s, z - s); glVertex3f(x + s, y - s, z + s);
//    }
//    glEnd();
//
//    glutSwapBuffers();
//}
//
//// --------------------------------------------------------
//// 鍵盤控制 (Keyboard)
//// W/S: 上下移動, A/D: 左右移動, Q/E: 前後縮放
//// --------------------------------------------------------
//void Keyboard(unsigned char key, int x, int y)
//{
//    float speed = 50.0f * SCALE_FACTOR; // 移動速度
//
//    switch (key)
//    {
//    case 'w': y_shift += speed; break;
//    case 's': y_shift -= speed; break;
//    case 'd': x_shift += speed; break;
//    case 'a': x_shift -= speed; break;
//    case 'q': z_shift += speed; break; // 拉近 (或推遠，視座標系而定)
//    case 'e': z_shift -= speed; break; // 推遠
//    case 27:  exit(0); break; // ESC 鍵離開
//    }
//    glutPostRedisplay();
//}
//
//// --------------------------------------------------------
//// 滑鼠點擊 (Mouse Click)
//// --------------------------------------------------------
//void Mouse(int button, int state, int x, int y)
//{
//    if (state == GLUT_DOWN) {
//        old_rot_x = x;
//        old_rot_y = y;
//    }
//    else if (state == GLUT_UP) {
//        record_x += x - old_rot_x;
//        record_y += y - old_rot_y;
//        rot_x = 0;
//        rot_y = 0;
//    }
//}
//
//// --------------------------------------------------------
//// 滑鼠拖曳 (Mouse Motion) -> 旋轉視角
//// --------------------------------------------------------
//void MotionMouse(int x, int y)
//{
//    rot_x = x - old_rot_x;
//    rot_y = y - old_rot_y;
//    glutPostRedisplay();
//}
//
//// --------------------------------------------------------
//// 視窗大小改變 (Window Reshape)
//// --------------------------------------------------------
//void WindowSize(int w, int h)
//{
//    if (h == 0) h = 1;
//    glViewport(0, 0, w, h);
//
//    glMatrixMode(GL_PROJECTION);
//    glLoadIdentity();
//
//    float aspect = (float)w / (float)h;
//    // 設定透視投影：視角 45 度, 最近 1.0, 最遠 100000.0 (確保看的到遠處的點)
//    gluPerspective(45, aspect, 1.0, 100000.0);
//
//    glMatrixMode(GL_MODELVIEW);
//    glLoadIdentity();
//}
//
//// --------------------------------------------------------
//// 主程式 (Main)
//// --------------------------------------------------------
//int main(int argc, char* argv[])
//{
//    // 1. 讀取 TXT 檔案
//    cout << "[Info] 正在讀取檔案: " << INPUT_TXT_PATH << " ..." << endl;
//
//    ifstream fin(INPUT_TXT_PATH);
//    if (!fin.is_open()) {
//        cerr << "[Error] 無法開啟檔案！請檢查路徑是否正確。" << endl;
//        system("pause");
//        return -1;
//    }
//
//    int x, y, z, r, g, b;
//    // 假設檔案格式為: X Y Z R G B (中間用空白分隔)
//    while (fin >> x >> y >> z >> r >> g >> b) {
//        VoxelPoint p;
//        p.X = x;
//        p.Y = y;
//        p.Z = z;
//        p.R = (unsigned char)r;
//        p.G = (unsigned char)g;
//        p.B = (unsigned char)b;
//        voxelData.push_back(p);
//    }
//    fin.close();
//
//    cout << "[Success] 讀取完成！共 " << voxelData.size() << " 個點。" << endl;
//    cout << "---------------------------------------------" << endl;
//    cout << "操作說明：" << endl;
//    cout << "  [滑鼠左鍵拖曳] 旋轉視角" << endl;
//    cout << "  [W / S] 上下移動" << endl;
//    cout << "  [A / D] 左右移動" << endl;
//    cout << "  [Q / E] 前後移動 (Zoom)" << endl;
//    cout << "  [ESC] 離開程式" << endl;
//
//    // 2. 初始化 OpenGL / GLUT
//    glutInit(&argc, argv);
//    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
//    glutInitWindowPosition(100, 100);
//    glutInitWindowSize(1024, 768);
//    glutCreateWindow("3D Voxel Viewer");
//
//    // 3. 註冊回呼函式
//    glutReshapeFunc(WindowSize);
//    glutDisplayFunc(Display);
//    glutKeyboardFunc(Keyboard);
//    glutMouseFunc(Mouse);
//    glutMotionFunc(MotionMouse);
//
//    // 4. 開始主迴圈
//    glutMainLoop();
//
//    return 0;
//}