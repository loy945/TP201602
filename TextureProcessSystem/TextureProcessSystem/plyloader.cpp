#include "stdafx.h"
#include "var.h"
#include "plyloader.h"
#include "Line.h"
#include "Triangle.h"
#include <fstream>
using namespace std;
Model_PLY::Model_PLY()
{
	Faces_Triangles = NULL;
	Faces_Quads = NULL;
	Vertex_Buffer = NULL;
	Normals = NULL;
	showPart=1;

}

Model_PLY::~Model_PLY()
{
	if (Vertex_Buffer != NULL)
	{
		free(Vertex_Buffer);
		Vertex_Buffer = NULL;
	}

	if (Faces_Triangles != NULL)
	{
		free(Faces_Triangles);
		Faces_Triangles = NULL;
	}

	if (Faces_Quads != NULL)
	{
		free(Faces_Quads);
		Faces_Quads = NULL;
	}

	if (Normals != NULL)
	{
		free(Normals);
		Normals = NULL;
	}

}

float* Model_PLY::calculateNormal(float *coord1, float *coord2, float *coord3)
{
	/* calculate Vector1 and Vector2 */
	float va[3], vb[3], vr[3], val;
	va[0] = coord1[0] - coord2[0];
	va[1] = coord1[1] - coord2[1];
	va[2] = coord1[2] - coord2[2];

	vb[0] = coord1[0] - coord3[0];
	vb[1] = coord1[1] - coord3[1];
	vb[2] = coord1[2] - coord3[2];

	/* cross product */
	vr[0] = va[1] * vb[2] - vb[1] * va[2];
	vr[1] = vb[0] * va[2] - va[0] * vb[2];
	vr[2] = va[0] * vb[1] - vb[0] * va[1];

	/* normalization factor */
	val = sqrt(vr[0] * vr[0] + vr[1] * vr[1] + vr[2] * vr[2]);

	float norm[3];
	norm[0] = vr[0] / val;
	norm[1] = vr[1] / val;
	norm[2] = vr[2] / val;


	return norm;
}
void Model_PLY::writeMesh(char *filename){
	int i = 0;
	FILE *out = fopen(filename, "w");
	fprintf(out, "%d\n", this->pointArry.size());
	fprintf(out, "%d\n", this->faceArry.size());
	for (i = 0; i<this->pointArry.size(); i++){
		fprintf(out, "%lf %lf %lf\n", pointArry[i].x, pointArry[i].y, pointArry[i].z);
	}
	for (i = 0; i<this->faceArry.size(); i++)
		fprintf(out, "3 %d %d %d\n", faceArry[i].ptnum[0], faceArry[i].ptnum[1], faceArry[i].ptnum[2]);
	fclose(out);
}
bool Model_PLY::Load(string filename)
{
	this->TotalConnectedTriangles = 0;
	this->TotalConnectedQuads = 0;
	this->TotalConnectedPoints = 0;


	if (filename.length() > 0)
	{
		FILE* file = fopen(filename.c_str(), "r");

		fseek(file, 0, SEEK_END);
		long fileSize = ftell(file);

		try
		{
			Vertex_Buffer = (float*)malloc(ftell(file));
		}
		catch (char*)
		{
			return false;
		}

		if (Vertex_Buffer == NULL) return false;

		fseek(file, 0, SEEK_SET);

		Faces_Triangles = (float*)malloc(fileSize*sizeof(float));
		Normals = (float*)malloc(fileSize*sizeof(float));

		if (file)
		{
			int i = 0;
			int temp = 0;
			int quads_index = 0;
			int triangle_index = 0;
			int normal_index = 0;
			char buffer[1000];


			fgets(buffer, 300, file);			// ply


			// READ HEADER
			// -----------------

			// Find number of vertexes
			while (strncmp("element vertex", buffer, strlen("element vertex")) != 0)
			{
				fgets(buffer, 300, file);			// format
			}

			strcpy(buffer, buffer + strlen("element vertex"));
			sscanf(buffer, "%i", &this->TotalConnectedPoints);


			// Find number of vertexes
			fseek(file, 0, SEEK_SET);
			while (strncmp("element face", buffer, strlen("element face")) != 0)
			{
				fgets(buffer, 300, file);			// format
			}
			strcpy(buffer, buffer + strlen("element face"));
			sscanf(buffer, "%i", &this->TotalFaces);


			// go to end_header
			while (strncmp("end_header", buffer, strlen("end_header")) != 0)
			{
				fgets(buffer, 300, file);			// format
			}

			//----------------------


			// read verteces
			i = 0; int pointnum = 0;
			for (int iterator = 0; iterator < this->TotalConnectedPoints; iterator++)
			{
				gl_point gl_vertexPt;
				gl_vertexPt.pointNum = pointnum;

				fgets(buffer, 300, file);

				sscanf(buffer, "%f %f %f", &gl_vertexPt.x, &gl_vertexPt.y, &gl_vertexPt.z);

				pointArry.push_back(gl_vertexPt);

				sscanf(buffer, "%f %f %f", &Vertex_Buffer[i], &Vertex_Buffer[i + 1], &Vertex_Buffer[i + 2]);
				i += 3;
				gl_vertexPt.beLongToFaceIndex.clear();
				pointnum++;
			}

			// read faces
			i = 0; int facenum = 0;
			for (int iterator = 0; iterator < this->TotalFaces; iterator++)
			{
				fgets(buffer, 300, file);

				

				if (buffer[0] == '3')
				{
					gl_face face;
					face.facenum = facenum;

					int vertex1 = 0, vertex2 = 0, vertex3 = 0;
					//sscanf(buffer,"%i%i%i\n", vertex1,vertex2,vertex3 );
					buffer[0] = ' ';

					sscanf(buffer, "%i%i%i", &face.ptnum[0], &face.ptnum[1], &face.ptnum[2]);
					face.isSetValue=false;
					face.value=0;

					
					

					vertex1 = face.ptnum[0];
					vertex2 = face.ptnum[1];
					vertex3 = face.ptnum[2];
					//给顶点添加信息，属于哪个三角形
					pointArry.at(face.ptnum[0]).beLongToFaceIndex.push_back(facenum);
					pointArry.at(face.ptnum[1]).beLongToFaceIndex.push_back(facenum);
					pointArry.at(face.ptnum[2]).beLongToFaceIndex.push_back(facenum);

					//sscanf(buffer, "%i%i%i", &vertex1, &vertex2, &vertex3);
					
					Faces_Triangles[triangle_index] = Vertex_Buffer[3 * vertex1];
					Faces_Triangles[triangle_index + 1] = Vertex_Buffer[3 * vertex1 + 1];
					Faces_Triangles[triangle_index + 2] = Vertex_Buffer[3 * vertex1 + 2];
					Faces_Triangles[triangle_index + 3] = Vertex_Buffer[3 * vertex2];
					Faces_Triangles[triangle_index + 4] = Vertex_Buffer[3 * vertex2 + 1];
					Faces_Triangles[triangle_index + 5] = Vertex_Buffer[3 * vertex2 + 2];
					Faces_Triangles[triangle_index + 6] = Vertex_Buffer[3 * vertex3];
					Faces_Triangles[triangle_index + 7] = Vertex_Buffer[3 * vertex3 + 1];
					Faces_Triangles[triangle_index + 8] = Vertex_Buffer[3 * vertex3 + 2];

					float coord1[3] = { Faces_Triangles[triangle_index], Faces_Triangles[triangle_index + 1], Faces_Triangles[triangle_index + 2] };
					float coord2[3] = { Faces_Triangles[triangle_index + 3], Faces_Triangles[triangle_index + 4], Faces_Triangles[triangle_index + 5] };
					float coord3[3] = { Faces_Triangles[triangle_index + 6], Faces_Triangles[triangle_index + 7], Faces_Triangles[triangle_index + 8] };

					
					/*face.corex=(pointArry.at(face.ptnum[0]).x;+coord2[0]+coord3[0])/3.0;
					face.corey=(coord1[1]+coord2[1]+coord3[1])/3.0;
					face.corez=(coord1[2]+coord2[2]+coord3[2])/3.0;
					*/
					float coreX = 0, coreY = 0, coreZ = 0;
					for (int i = 0; i < 3; i++)
					{
						coreX +=pointArry.at(face.ptnum[i]).x;
						coreY += pointArry.at(face.ptnum[i]).y;
						coreZ += pointArry.at(face.ptnum[i]).z;
					}
					face.corex = coreX / 3.0;
					face.corey = coreY / 3.0;
					face.corez = coreZ / 3.0;

					faceArry.push_back(face);




					float *norm = this->calculateNormal(coord1, coord2, coord3);
					

					Normals[normal_index] = norm[0];
					Normals[normal_index + 1] = norm[1];
					Normals[normal_index + 2] = norm[2];
					Normals[normal_index + 3] = norm[0];
					Normals[normal_index + 4] = norm[1];
					Normals[normal_index + 5] = norm[2];
					Normals[normal_index + 6] = norm[0];
					Normals[normal_index + 7] = norm[1];
					Normals[normal_index + 8] = norm[2];

					normal_index += 9;

					triangle_index += 9;
					TotalConnectedTriangles += 3;

					facenum++;
				}


				i += 3;
			}


			fclose(file);

			return true;
		}

		else 
		{ 
			MessageBox(NULL, _T("文件无法读取"), _T("错误"), MB_OK);
			return false;
		}
	}
	else 
	{
		MessageBox(NULL,_T("文件格式错误"),_T("错误"),MB_OK);
		return false;
	}
	return false;
}
void Model_PLY::Draw()
{
	/*glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, Faces_Triangles);
	glNormalPointer(GL_FLOAT, 0, Normals);
	glDrawArrays(GL_TRIANGLES, 0, TotalConnectedTriangles);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);*/
	return;
	vector<gl_face> * Triangle=&(faceArry);
	vector<gl_point> * Vertex=&(pointArry);
	vector<gl_point2d> * Vertex2d=&(point2DArry);
	if (pointArry[Triangle->at(0).ptnum[0]].x<-4e+8)
	{
		AfxMessageBox("model data is null");
	exit(-1);
	}
	int i;
	int a=3;
	glClear(GL_COLOR_BUFFER_BIT);
	glBegin(GL_TRIANGLES);//显示模型的面
	for(int i=0;i<faceArry.size();i++)
	{
		float v1x=pointArry[Triangle->at(i).ptnum[0]].x;
		float v1y=pointArry[Triangle->at(i).ptnum[0]].y;
		float v1z=pointArry[Triangle->at(i).ptnum[0]].z;

		float v2x=pointArry[Triangle->at(i).ptnum[1]].x;
		float v2y=pointArry[Triangle->at(i).ptnum[1]].y;
		float v2z=pointArry[Triangle->at(i).ptnum[1]].z;

		float v3x=pointArry[Triangle->at(i).ptnum[2]].x;
		float v3y=pointArry[Triangle->at(i).ptnum[2]].y;
		float v3z=pointArry[Triangle->at(i).ptnum[2]].z;

		if (Triangle->at(i).texCoords.size()==0)
		{
			if(v1x<-4e+8)
				break;
			if(Triangle->at(i).isShowColorIn3D)
			{
				glColor3f(Triangle->at(i).r,Triangle->at(i).g,Triangle->at(i).b);
			}else
			{
				glColor3f(0.5,0.5,0.5);//灰色
			}
			if(Triangle->at(i).renderTime==showPart-1||showPart==1)
			{
				glVertex3f(v1x,v1y,v1z);
				glVertex3f(v2x,v2y,v2z);
				glVertex3f(v3x,v3y,v3z);
			}
		}
	}
	glEnd();

	glEnable(GL_TEXTURE_2D);
	glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_REPLACE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glBegin(GL_TRIANGLES );//显示
	for(int i=0;i<faceArry.size();i++)
	{	
		if (Triangle->at(i).texCoords.size()>0)
		{
			glColor4f(1, 1, 1, 1);

			float v1x = pointArry[Triangle->at(i).ptnum[0]].x;
			float v1y = pointArry[Triangle->at(i).ptnum[0]].y;
			float v1z = pointArry[Triangle->at(i).ptnum[0]].z;

			float v2x = pointArry[Triangle->at(i).ptnum[1]].x;
			float v2y = pointArry[Triangle->at(i).ptnum[1]].y;
			float v2z = pointArry[Triangle->at(i).ptnum[1]].z;

			float v3x = pointArry[Triangle->at(i).ptnum[2]].x;
			float v3y = pointArry[Triangle->at(i).ptnum[2]].y;
			float v3z = pointArry[Triangle->at(i).ptnum[2]].z;


			for (int j = 0; j < Triangle->at(i).texCoords.size(); j++)
			{
				/*	glTexCoord2f(Triangle->at(i).texCoords[j]->cor[0][0], Triangle->at(i).texCoords[j]->cor[0][1]);

					glVertex3f(v1x, v1y, v1z);

					glTexCoord2f(Triangle->at(i).texCoords[j]->cor[1][0], Triangle->at(i).texCoords[j]->cor[1][1]);

					glVertex3f(v2x, v2y, v2z);

					glTexCoord2f(Triangle->at(i).texCoords[j]->cor[2][0], Triangle->at(i).texCoords[j]->cor[2][1]);

					glVertex3f(v3x, v3y, v3z);
					}*/
				glMultiTexCoord2f(texName, Triangle->at(i).texCoords[j]->cor[0][0], Triangle->at(i).texCoords[j]->cor[0][1]);

				glVertex3f(v1x, v1y, v1z);

				glMultiTexCoord2f(texName, Triangle->at(i).texCoords[j]->cor[1][0], Triangle->at(i).texCoords[j]->cor[1][1]);

				glVertex3f(v2x, v2y, v2z);

				glMultiTexCoord2f(texName, Triangle->at(i).texCoords[j]->cor[2][0], Triangle->at(i).texCoords[j]->cor[2][1]);

				glVertex3f(v3x, v3y, v3z);
			}
		}
			
	}
	glEnd();
	glFlush();
	glDisable(GL_TEXTURE_2D);


}
void gl_face::updateTexCoord()
{
	int i = 0;
	int j = 0;

	ofstream f("badtexlog.txt", ios::app);

		f << "updateTexCoord Triangle  " << this->facenum << endl;
		for (int k = 0; k < this->texCoords.size(); k++)
		{
			f << this->texCoords[k]->cor[0][0] << "," << this->texCoords[k]->cor[0][1] << endl;
			f << this->texCoords[k]->cor[1][0] << "," << this->texCoords[k]->cor[1][1] << endl;
			f << this->texCoords[k]->cor[2][0] << "," << this->texCoords[k]->cor[2][1] << endl;
		}
		f << endl;
		f.close();
	
	if (!texCoord.update)
	{
		texCoord.update = true;
		//判断是否
		bool addIn = false;
		
		for (int i = 0; i < 3; i++)
		{
			if (texCoord.cor[i][0] <= 1 && texCoord.cor[i][0] >= 0 && texCoord.cor[i][1] <= 1 && texCoord.cor[i][1] >= 0)
			{
				addIn = true;
				ofstream f("badtexlog.txt", ios::app);
				f << "GOOD1 Triangle  " << this->facenum << endl;
				for (int k = 0; k < this->texCoords.size(); k++)
				{
					f << this->texCoords[k]->cor[0][0] << "," << this->texCoords[k]->cor[0][1] << endl;
					f << this->texCoords[k]->cor[1][0] << "," << this->texCoords[k]->cor[1][1] << endl;
					f << this->texCoords[k]->cor[2][0] << "," << this->texCoords[k]->cor[2][1] << endl;
				}
				f << endl;
				f.close();
				i = 3;
			}
		}
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				if (!(texCoord.cor[i][j] == texCoord.cor[i][j] ))
				{
					ofstream f("badtexlog.txt", ios::app);
					f << "BAD1 Triangle  " << this->facenum << endl;
					for (int k = 0; k < this->texCoords.size(); k++)
					{
						f << this->texCoords[k]->cor[0][0] << "," << this->texCoords[k]->cor[0][1] << endl;
						f << this->texCoords[k]->cor[1][0] << "," << this->texCoords[k]->cor[1][1] << endl;
						f << this->texCoords[k]->cor[2][0] << "," << this->texCoords[k]->cor[2][1] << endl;
					}
					f << endl;
					f.close();
					return;
					//越界
				}
			}
		}
		//判断是否越界
		Line * l[3];
		for (i = 0; i < 3; i++)
		{
			l[i] = new Line();
		}
		l[0]->start.setValue(texCoord.cor[0][0], texCoord.cor[0][1], 0);
		l[0]->  end.setValue(texCoord.cor[1][0], texCoord.cor[1][1],0);
		l[1]->start.setValue(texCoord.cor[1][0], texCoord.cor[1][1], 0);
		l[1]->end.setValue(texCoord.cor[2][0], texCoord.cor[2][1], 0);
		l[2]->start.setValue(texCoord.cor[2][0], texCoord.cor[2][1], 0);
		l[2]->end.setValue(texCoord.cor[0][0], texCoord.cor[0][1], 0);
		Line * lb[4];
		for (i = 0; i < 4; i++)
		{
			lb[i] = new Line();	
		}
		lb[0]->start.setValue(0, 0, 0);
		lb[1]->start.setValue(0, 1, 0);
		lb[2]->start.setValue(1, 1, 0);
		lb[3]->start.setValue(1, 0, 0);
		lb[0]->end.setValue(0, 1, 0);
		lb[1]->end.setValue(1, 1, 0);
		lb[2]->end.setValue(1, 0, 0);
		lb[3]->end.setValue(0, 0, 0);

		for (i = 0; i < 4; i++)
		{
			for (j = 0; j < 3; j++)
			{
				if (lb[i]->isCross(*l[j]))
				{
					addIn = true;
					
					f << "GOOD2 Triangle  " << this->facenum << endl;
					for (int k = 0; k < this->texCoords.size(); k++)
					{
						f << this->texCoords[k]->cor[0][0] << "," << this->texCoords[k]->cor[0][1] << endl;
						f << this->texCoords[k]->cor[1][0] << "," << this->texCoords[k]->cor[1][1] << endl;
						f << this->texCoords[k]->cor[2][0] << "," << this->texCoords[k]->cor[2][1] << endl;
					}
					f << endl;
					f.close();
					j = 3;
					i = 4;
				}
			}
		}
		Point3D pt[3];
		for (i = 0; i < 3; i++)
		{
			pt[i].x = texCoord.cor[i][0];
			pt[i].y = texCoord.cor[i][1];
			pt[i].z = texCoord.cor[i][2];
		}
		Triangle * tri1 = new Triangle();
		tri1->setValue(pt);

		Point3D ptb[4];
		ptb[0].setValue(Point3D(0, 0, 0));
		ptb[1].setValue(Point3D(0, 1, 0));
		ptb[2].setValue(Point3D(1, 1, 0));
		ptb[3].setValue(Point3D(1, 0, 0));
		if (!addIn)
		{
			for (i = 0; i < 4; i++)
			{
				if (!tri1->isAPointInTriangle(ptb[i]))
				{
					ofstream f("badtexlog.txt", ios::app);
					f << "BAD2 Triangle  " << this->facenum << endl;
					for (int k = 0; k < this->texCoords.size(); k++)
					{
						f << this->texCoords[k]->cor[0][0] << "," << this->texCoords[k]->cor[0][1] << endl;
						f << this->texCoords[k]->cor[1][0] << "," << this->texCoords[k]->cor[1][1] << endl;
						f << this->texCoords[k]->cor[2][0] << "," << this->texCoords[k]->cor[2][1] << endl;
					}
					f << endl;
					f.close();
					return;
				}
			}
			addIn = true;
			ofstream f("badtexlog.txt", ios::app);
			f << "GOOD3 Triangle  " << this->facenum << endl;
			for (int k = 0; k < this->texCoords.size(); k++)
			{
				f << this->texCoords[k]->cor[0][0] << "," << this->texCoords[k]->cor[0][1] << endl;
				f << this->texCoords[k]->cor[1][0] << "," << this->texCoords[k]->cor[1][1] << endl;
				f << this->texCoords[k]->cor[2][0] << "," << this->texCoords[k]->cor[2][1] << endl;
			}
			f << endl;
			f.close();
		}

		if (!addIn) return;
		
		//更新纹理信息，压入队列
		Point3fv3 * pt3fv3 = new Point3fv3();
		pt3fv3->setValue(texCoord);
		texCoords.push_back(pt3fv3);
		
	}
}