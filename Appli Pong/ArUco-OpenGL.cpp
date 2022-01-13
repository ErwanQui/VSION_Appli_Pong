//
//  ArUco-OpenGL.cpp
//
//  Created by Jean-Marie Normand on 28/02/13.
//  Copyright (c) 2013 Centrale Nantes. All rights reserved.
//


#include "ArUco-OpenGL.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2\calib3d.hpp>
#include <cmath>

Point2f posBall = Point2f::Point_(5.0f, -2.5f);
Point2f vitBall = Point2f::Point_(1.0f, 1.0f);
Point2f posJ1;
Point2f posJ2;
float vitesse = 0.05f;

double positionJ1[3];
double orientationJ1[4];
double positionJ2[3];
double orientationJ2[4];
double positionA[3];
double orientationA[4];

double modelview_matrixA[16];
double modelview_matrixJ1[16];
double modelview_matrixJ2[16];

// Constructor
ArUco::ArUco(string intrinFileName, float markerSize) {
   // Initializing attributes
   m_IntrinsicFile= intrinFileName;
   m_MarkerSize   = markerSize;
   // read camera parameters if passed
   m_CameraParams.readFromXMLFile(intrinFileName);
  
}

// Destructor
ArUco::~ArUco() {}

void ArUco::resizeCameraParams(cv::Size newSize) {
	m_CameraParams.resize(newSize);
}

// Detect marker and draw things
void ArUco::doWork(Mat inputImg) {
   m_InputImage   = inputImg;
   m_GlWindowSize = m_InputImage.size();
   m_CameraParams.resize(m_InputImage.size());
   resize(m_GlWindowSize.width, m_GlWindowSize.height);
}

// Draw axis function
void ArUco::drawAxis(float axisSize) {
   // X
   glColor3f (1,0,0);
   glBegin(GL_LINES);
   glVertex3f(0.0f, 0.0f, 0.0f); // origin of the line
   glVertex3f(axisSize,0.0f, 0.0f); // ending point of the line
   glEnd( );
   
   // Y
   glColor3f (0,1,0);
   glBegin(GL_LINES);
   glVertex3f(0.0f, 0.0f, 0.0f); // origin of the line
   glVertex3f(0.0f, axisSize, 0.0f); // ending point of the line
   glEnd( );
   
   // Z
   glColor3f (0,0,1);
   glBegin(GL_LINES);
   glVertex3f(0.0f, 0.0f, 0.0f); // origin of the line
   glVertex3f(0.0f, 0.0f, axisSize); // ending point of the line
   glEnd( );
}

void ArUco::drawWireCube(GLdouble size) {
    drawBox(size, GL_LINE_LOOP);
}

// Fonction qui dessine un cube de différentes manières (type)
void ArUco::drawBox(GLfloat size, GLenum type)
{
		static const GLfloat n[6][3] =
		{
		  {-1.0, 0.0, 0.0},
		  {0.0, 1.0, 0.0},
		  {1.0, 0.0, 0.0},
		  {0.0, -1.0, 0.0},
		  {0.0, 0.0, 1.0},
		  {0.0, 0.0, -1.0}
		};
		static const GLint faces[6][4] =
		{
		  {0, 1, 2, 3},
		  {3, 2, 6, 7},
		  {7, 6, 5, 4},
		  {4, 5, 1, 0},
		  {5, 6, 2, 1},
		  {7, 4, 0, 3}
		};
		GLfloat v[8][3];
		GLint i;

		v[0][0] = v[1][0] = v[2][0] = v[3][0] = -size / 2;
		v[4][0] = v[5][0] = v[6][0] = v[7][0] = size / 2;
		v[0][1] = v[1][1] = v[4][1] = v[5][1] = -size / 2;
		v[2][1] = v[3][1] = v[6][1] = v[7][1] = size / 2;
		v[0][2] = v[3][2] = v[4][2] = v[7][2] = -size / 2;
		v[1][2] = v[2][2] = v[5][2] = v[6][2] = size / 2;

		for (i = 5; i >= 0; i--) {
			glBegin(type);
			glNormal3fv(&n[i][0]);
			glVertex3fv(&v[faces[i][0]][0]);
			glVertex3fv(&v[faces[i][1]][0]);
			glVertex3fv(&v[faces[i][2]][0]);
			glVertex3fv(&v[faces[i][3]][0]);
			glEnd();
		}
}

bool ArUco::collider(GLfloat J[4][2], GLfloat size)
{
	float posInitx = (positionA[0] + posBall.x * size - positionJ1[0]);
	float posInity = (positionA[1] + posBall.y * size - positionJ1[1]);
	if ((posInitx - J[0][0]) * (J[1][1] - J[0][1]) / (J[1][0] - J[0][0]) + J[0][1] <= posInity) {
		if ((posInitx - J[2][0]) * (J[3][1] - J[2][1]) / (J[3][0] - J[2][0]) + J[2][1] >= posInity) {
			if ((posInitx - J[0][0]) * (J[3][1] - J[0][1]) / (J[3][0] - J[0][0]) + J[0][1] <= posInity) {
				if ((posInitx - J[2][0]) * (J[1][1] - J[2][1]) / (J[1][0] - J[2][0]) + J[2][1] >= posInity) {
					return true;
				}
			}
			else {
				if ((posInitx - J[2][0]) * (J[1][1] - J[2][1]) / (J[1][0] - J[2][0]) + J[2][1] <= posInity) {
					return true;
				}
			}
		}
	}
	else {
		if ((posInitx - J[2][0]) * (J[3][1] - J[2][1]) / (J[3][0] - J[2][0]) + J[2][1] <= posInity) {
			if ((posInitx - J[0][0]) * (J[3][1] - J[0][1]) / (J[3][0] - J[0][0]) + J[0][1] <= posInity) {
				if ((posInitx - J[2][0]) * (J[1][1] - J[2][1]) / (J[1][0] - J[2][0]) + J[2][1] >= posInity) {
					return true;
				}
			}
			else {
				if ((posInitx - J[2][0]) * (J[1][1] - J[2][1]) / (J[1][0] - J[2][0]) + J[2][1] <= posInity) {
					return true;
				}
			}
		}
	}
	return false;
}

void ArUco::drawAreneComplexe(GLfloat size, GLenum type)
{
	float longueur = 10 * size;
	float largeur = 5 * size;
	float epaisseur = size / 8;
	float taille = size / 4;
	float alpha1 = orientationJ1[0];
	float alpha2 = orientationJ2[0];

	GLfloat J1[4][2] =
	{
		{-size * sin(alpha1) + taille * cos(alpha1), size * cos(alpha1) + taille * sin(alpha1)},
		{-size * sin(alpha1) - taille * cos(alpha1), size* cos(alpha1) - taille * sin(alpha1)},
		{size * sin(alpha1) - taille * cos(alpha1), -size* cos(alpha1) - taille * sin(alpha1)},
		{size * sin(alpha1) + taille * cos(alpha1), -size * cos(alpha1) + taille * sin(alpha1)}
	};

	GLfloat J2[4][2] =
	{
		{-size * sin(alpha2) + taille * cos(alpha2), size * cos(alpha2) + taille * sin(alpha2)},
		{-size * sin(alpha2) - taille * cos(alpha2), size * cos(alpha2) - taille * sin(alpha2)},
		{size * sin(alpha2) - taille * cos(alpha2), -size * cos(alpha2) - taille * sin(alpha2)},
		{size * sin(alpha2) + taille * cos(alpha2), -size * cos(alpha2) + taille * sin(alpha2)}
	};


	static const GLfloat n[6][3] =
	{
	  {-1.0, 0.0, 0.0},
	  {0.0, 1.0, 0.0},
	  {1.0, 0.0, 0.0},
	  {0.0, -1.0, 0.0},
	  {0.0, 0.0, 1.0},
	  {0.0, 0.0, -1.0}
	};
	static const GLint faces[6][4] =
	{
	  {0, 1, 2, 3},
	  {3, 2, 6, 7},
	  {7, 6, 5, 4},
	  {4, 5, 1, 0},
	  {5, 6, 2, 1},
	  {7, 4, 0, 3}
	};
	GLfloat v[8][3];
	GLint i;

	v[0][0] = v[1][0] = v[2][0] = v[3][0] = -epaisseur;
	v[4][0] = v[5][0] = v[6][0] = v[7][0] = epaisseur;
	v[0][1] = v[1][1] = v[4][1] = v[5][1] = -largeur;
	v[2][1] = v[3][1] = v[6][1] = v[7][1] = 0;
	v[0][2] = v[3][2] = v[4][2] = v[7][2] = -size / 2;
	v[1][2] = v[2][2] = v[5][2] = v[6][2] = -size / 2;

	for (i = 5; i >= 0; i--) {
		glBegin(type);
		glNormal3fv(&n[i][0]);
		glVertex3fv(&v[faces[i][0]][0]);
		glVertex3fv(&v[faces[i][1]][0]);
		glVertex3fv(&v[faces[i][2]][0]);
		glVertex3fv(&v[faces[i][3]][0]);
		glEnd();
	}

	glTranslatef(longueur, 0, 0);

	v[0][0] = v[1][0] = v[2][0] = v[3][0] = -epaisseur;
	v[4][0] = v[5][0] = v[6][0] = v[7][0] = epaisseur;
	v[0][1] = v[1][1] = v[4][1] = v[5][1] = -largeur;
	v[2][1] = v[3][1] = v[6][1] = v[7][1] = 0;
	v[0][2] = v[3][2] = v[4][2] = v[7][2] = -size / 2;
	v[1][2] = v[2][2] = v[5][2] = v[6][2] = -size / 2;

	for (i = 5; i >= 0; i--) {
		glBegin(type);
		glNormal3fv(&n[i][0]);
		glVertex3fv(&v[faces[i][0]][0]);
		glVertex3fv(&v[faces[i][1]][0]);
		glVertex3fv(&v[faces[i][2]][0]);
		glVertex3fv(&v[faces[i][3]][0]);
		glEnd();
	}

	v[0][0] = v[1][0] = v[2][0] = v[3][0] = -longueur - epaisseur;
	v[4][0] = v[5][0] = v[6][0] = v[7][0] = epaisseur;
	v[0][1] = v[1][1] = v[4][1] = v[5][1] = -epaisseur;
	v[2][1] = v[3][1] = v[6][1] = v[7][1] = epaisseur;
	v[0][2] = v[3][2] = v[4][2] = v[7][2] = -size / 2;
	v[1][2] = v[2][2] = v[5][2] = v[6][2] = -size / 2;

	for (i = 5; i >= 0; i--) {
		glBegin(type);
		glNormal3fv(&n[i][0]);
		glVertex3fv(&v[faces[i][0]][0]);
		glVertex3fv(&v[faces[i][1]][0]);
		glVertex3fv(&v[faces[i][2]][0]);
		glVertex3fv(&v[faces[i][3]][0]);
		glEnd();
	}

	glTranslatef(0, -largeur, 0);

	v[0][0] = v[1][0] = v[2][0] = v[3][0] = -longueur - epaisseur;
	v[4][0] = v[5][0] = v[6][0] = v[7][0] = epaisseur;
	v[0][1] = v[1][1] = v[4][1] = v[5][1] = -epaisseur;
	v[2][1] = v[3][1] = v[6][1] = v[7][1] = epaisseur;
	v[0][2] = v[3][2] = v[4][2] = v[7][2] = -size / 2;
	v[1][2] = v[2][2] = v[5][2] = v[6][2] = -size / 2;

	for (i = 5; i >= 0; i--) {
		glBegin(type);
		glNormal3fv(&n[i][0]);
		glVertex3fv(&v[faces[i][0]][0]);
		glVertex3fv(&v[faces[i][1]][0]);
		glVertex3fv(&v[faces[i][2]][0]);
		glVertex3fv(&v[faces[i][3]][0]);
		glEnd();
	}
	cout << endl << posBall.x * size << "  " << posBall.y * size << "  " << positionA[0] << "  " << positionA[1] << "  " << positionJ1[0] << "  " << positionJ1[1] << endl;
	if (posBall.x * size <= epaisseur || posBall.x * size >= longueur - epaisseur) { vitBall.x *= -1; } // murs verticaux
	if (posBall.y * size <= -largeur - epaisseur || posBall.y * size >= -epaisseur) { vitBall.y *= -1; } // murs horizontaux
	if (collider(J1, size)) {
		vitBall.x *= -1;
		vitBall.y *= -1;
	}
	if (collider(J2, size)) {
		vitBall.x *= -1;
		vitBall.y *= -1;
	}
	posBall.x += vitBall.x * vitesse;
	posBall.y += vitBall.y * vitesse;
	glTranslatef(-longueur + posBall.x * size, largeur + posBall.y * size, 0);
	glColor3f(0, 1, 0);
	drawBox(m_MarkerSize * 0.1, GL_QUADS);
}

void ArUco::drawArene(GLfloat size, GLenum type)
{
	float longueur = 10 * size;
	float largeur = 5 * size;
	float epaisseur = size / 8;
	float taille = size / 4;

	static const GLfloat n[6][3] =
	{
	  {-1.0, 0.0, 0.0},
	  {0.0, 1.0, 0.0},
	  {1.0, 0.0, 0.0},
	  {0.0, -1.0, 0.0},
	  {0.0, 0.0, 1.0},
	  {0.0, 0.0, -1.0}
	};
	static const GLint faces[6][4] =
	{
	  {0, 1, 2, 3},
	  {3, 2, 6, 7},
	  {7, 6, 5, 4},
	  {4, 5, 1, 0},
	  {5, 6, 2, 1},
	  {7, 4, 0, 3}
	};
	GLfloat v[8][3];
	GLint i;

	v[0][0] = v[1][0] = v[2][0] = v[3][0] = -epaisseur;
	v[4][0] = v[5][0] = v[6][0] = v[7][0] = epaisseur;
	v[0][1] = v[1][1] = v[4][1] = v[5][1] = -largeur;
	v[2][1] = v[3][1] = v[6][1] = v[7][1] = 0;
	v[0][2] = v[3][2] = v[4][2] = v[7][2] = -size / 2;
	v[1][2] = v[2][2] = v[5][2] = v[6][2] = -size / 2;

	for (i = 5; i >= 0; i--) {
		glBegin(type);
		glNormal3fv(&n[i][0]);
		glVertex3fv(&v[faces[i][0]][0]);
		glVertex3fv(&v[faces[i][1]][0]);
		glVertex3fv(&v[faces[i][2]][0]);
		glVertex3fv(&v[faces[i][3]][0]);
		glEnd();
	}

	glTranslatef(longueur, 0, 0);

	v[0][0] = v[1][0] = v[2][0] = v[3][0] = -epaisseur;
	v[4][0] = v[5][0] = v[6][0] = v[7][0] = epaisseur;
	v[0][1] = v[1][1] = v[4][1] = v[5][1] = -largeur;
	v[2][1] = v[3][1] = v[6][1] = v[7][1] = 0;
	v[0][2] = v[3][2] = v[4][2] = v[7][2] = -size / 2;
	v[1][2] = v[2][2] = v[5][2] = v[6][2] = -size / 2;

	for (i = 5; i >= 0; i--) {
		glBegin(type);
		glNormal3fv(&n[i][0]);
		glVertex3fv(&v[faces[i][0]][0]);
		glVertex3fv(&v[faces[i][1]][0]);
		glVertex3fv(&v[faces[i][2]][0]);
		glVertex3fv(&v[faces[i][3]][0]);
		glEnd();
	}

	v[0][0] = v[1][0] = v[2][0] = v[3][0] = -longueur - epaisseur;
	v[4][0] = v[5][0] = v[6][0] = v[7][0] = epaisseur;
	v[0][1] = v[1][1] = v[4][1] = v[5][1] = -epaisseur;
	v[2][1] = v[3][1] = v[6][1] = v[7][1] = epaisseur;
	v[0][2] = v[3][2] = v[4][2] = v[7][2] = -size / 2;
	v[1][2] = v[2][2] = v[5][2] = v[6][2] = -size / 2;

	for (i = 5; i >= 0; i--) {
		glBegin(type);
		glNormal3fv(&n[i][0]);
		glVertex3fv(&v[faces[i][0]][0]);
		glVertex3fv(&v[faces[i][1]][0]);
		glVertex3fv(&v[faces[i][2]][0]);
		glVertex3fv(&v[faces[i][3]][0]);
		glEnd();
	}

	glTranslatef(0, -largeur, 0);

	v[0][0] = v[1][0] = v[2][0] = v[3][0] = -longueur - epaisseur;
	v[4][0] = v[5][0] = v[6][0] = v[7][0] = epaisseur;
	v[0][1] = v[1][1] = v[4][1] = v[5][1] = -epaisseur;
	v[2][1] = v[3][1] = v[6][1] = v[7][1] = epaisseur;
	v[0][2] = v[3][2] = v[4][2] = v[7][2] = -size / 2;
	v[1][2] = v[2][2] = v[5][2] = v[6][2] = -size / 2;

	for (i = 5; i >= 0; i--) {
		glBegin(type);
		glNormal3fv(&n[i][0]);
		glVertex3fv(&v[faces[i][0]][0]);
		glVertex3fv(&v[faces[i][1]][0]);
		glVertex3fv(&v[faces[i][2]][0]);
		glVertex3fv(&v[faces[i][3]][0]);
		glEnd();
	}

	if (posBall.x * size <= epaisseur || posBall.x * size >= longueur - epaisseur) { vitBall.x *= -1; } // murs verticaux
	if (posBall.y * size <= -largeur - epaisseur || posBall.y * size >= -epaisseur) { vitBall.y *= -1; } // murs horizontaux

	if ((posBall.x * size + positionA[0] - positionJ1[0] <= size && posBall.x * size + positionA[0] - positionJ1[0] >= -size)
		&& (posBall.y * size + positionA[1] - positionJ1[1] <= size && posBall.y * size + positionA[1] - positionJ1[1] >= -size))
	{
		vitBall.x *= -1;
	}
	else if ((posBall.x * size + positionA[0] - positionJ2[0] <= size && posBall.x * size + positionA[0] - positionJ2[0] >= -size)
		&& (posBall.y * size + positionA[1] - positionJ2[1] <= size && posBall.y * size + positionA[1] - positionJ2[1] >= -size))
	{
		vitBall.x *= -1;
	}
	posBall.x += vitBall.x * vitesse;
	posBall.y += vitBall.y * vitesse;
	glTranslatef(-longueur + posBall.x * size, largeur + posBall.y * size, 0);
	glColor3f(0, 1, 0);
	drawBox(m_MarkerSize * 0.1, GL_QUADS);
}

void ArUco::drawPlayer(GLfloat size, GLenum type, bool player)
{
	static const GLfloat n[6][3] =
	{
	  {-1.0, 0.0, 0.0},
	  {0.0, 1.0, 0.0},
	  {1.0, 0.0, 0.0},
	  {0.0, -1.0, 0.0},
	  {0.0, 0.0, 1.0},
	  {0.0, 0.0, -1.0}
	};
	static const GLint faces[6][4] =
	{
	  {0, 1, 2, 3},
	  {3, 2, 6, 7},
	  {7, 6, 5, 4},
	  {4, 5, 1, 0},
	  {5, 6, 2, 1},
	  {7, 4, 0, 3}
	};
	GLfloat v[8][3];
	GLint i;

	v[0][0] = v[1][0] = v[2][0] = v[3][0] = -size / 4;
	v[4][0] = v[5][0] = v[6][0] = v[7][0] = size / 4;
	v[0][1] = v[1][1] = v[4][1] = v[5][1] = -size;
	v[2][1] = v[3][1] = v[6][1] = v[7][1] = size;
	v[0][2] = v[3][2] = v[4][2] = v[7][2] = -size / 2;
	v[1][2] = v[2][2] = v[5][2] = v[6][2] = size / 8;

	for (i = 5; i >= 0; i--) {
		glColor3f(1, 1, 1);
		glBegin(type);
		glNormal3fv(&n[i][0]);
		glVertex3fv(&v[faces[i][0]][0]);
		glVertex3fv(&v[faces[i][1]][0]);
		glVertex3fv(&v[faces[i][2]][0]);
		glVertex3fv(&v[faces[i][3]][0]);
		glEnd();
		if (player) { glColor3f(0, 0, 1); }
		else { glColor3f(1, 0, 0); }
		glBegin(GL_LINE_LOOP);
		glNormal3fv(&n[i][0]);
		glVertex3fv(&v[faces[i][0]][0]);
		glVertex3fv(&v[faces[i][1]][0]);
		glVertex3fv(&v[faces[i][2]][0]);
		glVertex3fv(&v[faces[i][3]][0]);
		glEnd();
	}
}

// GLUT functionnalities

// Drawing function
void ArUco::drawScene() {
   // If we do not have an image we don't do anyhting
   if (m_ResizedImage.rows==0)
      return;
   
   // On "reset" les matrices OpenGL de ModelView et de Projection
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();

   // On deinit une vue orthographique de la taille de l'image OpenGL
   glOrtho(0, m_GlWindowSize.width, 0, m_GlWindowSize.height, -1.0, 1.0);
   // on definit le viewport correspond a un rendu "plein ecran"
   glViewport(0, 0, m_GlWindowSize.width , m_GlWindowSize.height);

   // on desactive les textures
   glDisable(GL_TEXTURE_2D);

   // On "flippe" l'axe des Y car OpenCV et OpenGL on un axe Y inverse pour les images/textures
   glPixelZoom( 1, -1);

   // On definit la position ou l'on va ecrire dans l'image
   glRasterPos3f(0, m_GlWindowSize.height, -1.0f);
   
   // On "dessine" les pixels contenus dans l'image OpenCV m_ResizedImage (donc l'image de la Webcam qui nous sert de fond)
   glDrawPixels (m_GlWindowSize.width, m_GlWindowSize.height, GL_RGB, GL_UNSIGNED_BYTE, m_ResizedImage.ptr(0));
   
   // On active ensuite le depth test pour les objets 3D
   glEnable(GL_DEPTH_TEST);
   
   // On passe en mode projection pour definir la bonne projection calculee par ArUco
   glMatrixMode(GL_PROJECTION);
   double proj_matrix[16];
   m_CameraParams.glGetProjectionMatrix(m_ResizedImage.size(),m_GlWindowSize,proj_matrix,0.01,100);
   glLoadIdentity();
   // on charge la matrice d'ArUco 
   glLoadMatrixd(proj_matrix);
   
   // On affiche le nombre de marqueurs (ne sert a rien)
   double modelview_matrix[16];
   std::cout << "Number of markers: " << m_Markers.size() << std::endl;
   
   // On desactive le depth test
   glDisable(GL_DEPTH_TEST);

   // Pour chaque marqueur detecte
   for (unsigned int m=0;m<m_Markers.size();m++)
   {
	   if (m_Markers[m].id == 217) { // Arene
		   // On recupere la matrice de modelview qui correspond au marqueur [m]
		   m_Markers[m].glGetModelViewMatrix(modelview_matrixA);
		   glMatrixMode(GL_MODELVIEW);
		   glLoadIdentity();
		   // on charge cette matrice pour se placer dans le repere de ce marqueur [m] 
		   glLoadMatrixd(modelview_matrixA);

		   m_Markers[m].OgreGetPoseParameters(positionA, orientationA);
		   // On dessine les axes X Y Z
		   drawAxis(m_MarkerSize);

		   // On se deplace sur Z de la moitie du marqueur pour dessiner "sur" le plan du marqueur
		   glTranslatef(0, 0, m_MarkerSize / 2.);

		   // On sauvegarde la matrice courante
		   glPushMatrix();

		   // on choisit une couleur
		   glColor3f(1, 1, 1);

		   //On dessine un cube en fil de fer
		   drawAreneComplexe(m_MarkerSize, GL_QUADS);

		   // On re=charge la matrice que l'on a sauvegarde
		   glPopMatrix();
	   }
	   if (m_Markers[m].id == 143) { // J1
		   // On recupere la matrice de modelview qui correspond au marqueur [m]
		   m_Markers[m].glGetModelViewMatrix(modelview_matrixJ1);
		   glMatrixMode(GL_MODELVIEW);
		   glLoadIdentity();
		   // on charge cette matrice pour se placer dans le repere de ce marqueur [m] 
		   glLoadMatrixd(modelview_matrixJ1);

		   m_Markers[m].OgreGetPoseParameters(positionJ1, orientationJ1);

		   // On dessine les axes X Y Z
		   drawAxis(m_MarkerSize);

		   // On se deplace sur Z de la moitie du marqueur pour dessiner "sur" le plan du marqueur
		   glTranslatef(0, 0, m_MarkerSize / 2.);

		   // On sauvegarde la matrice courante
		   glPushMatrix();

		   //On dessine un cube en fil de fer
		   drawPlayer(m_MarkerSize, GL_QUADS, false);

		   // On re=charge la matrice que l'on a sauvegarde
		   glPopMatrix();
	   }
	   if (m_Markers[m].id == 144) { // J2
		   // On recupere la matrice de modelview qui correspond au marqueur [m]
		   m_Markers[m].glGetModelViewMatrix(modelview_matrixJ2);
		   glMatrixMode(GL_MODELVIEW);
		   glLoadIdentity();
		   // on charge cette matrice pour se placer dans le repere de ce marqueur [m] 
		   glLoadMatrixd(modelview_matrixJ2);

		   m_Markers[m].OgreGetPoseParameters(positionJ2, orientationJ2);

		   // On dessine les axes X Y Z
		   drawAxis(m_MarkerSize);

		   // On se deplace sur Z de la moitie du marqueur pour dessiner "sur" le plan du marqueur
		   glTranslatef(0, 0, m_MarkerSize / 2.);

		   // On sauvegarde la matrice courante
		   glPushMatrix();

		   //On dessine un cube en fil de fer
		   drawPlayer(m_MarkerSize, GL_QUADS, true);

		   // On re=charge la matrice que l'on a sauvegarde
		   glPopMatrix();
	   }
   }
   
   // Desactivation du depth test
   glDisable(GL_DEPTH_TEST);
}



// Idle function
void ArUco::idle(Mat newImage) {
   // Getting new image
   m_InputImage = newImage.clone();
  
   // Undistort image based on distorsion parameters
   m_UndInputImage.create(m_InputImage.size(),CV_8UC3);
   
   //transform color that by default is BGR to RGB because windows systems do not allow reading BGR images with opengl properly
   cv::cvtColor(m_InputImage,m_InputImage,cv::COLOR_BGR2RGB);
   
   //remove distorion in image ==> does not work very well (the YML file is not that of my camera)
   //cv::undistort(m_InputImage,m_UndInputImage, m_CameraParams.CameraMatrix, m_CameraParams.Distorsion);
   m_UndInputImage = m_InputImage.clone();
   
   //resize the image to the size of the GL window
   cv::resize(m_UndInputImage,m_ResizedImage,m_GlWindowSize);

   //detect markers
   m_PPDetector.detect(m_ResizedImage, m_Markers, m_CameraParams, m_MarkerSize, false);

}

// Resize function
void ArUco::resize(GLsizei iWidth, GLsizei iHeight) {
   m_GlWindowSize=Size(iWidth,iHeight);
   
   //not all sizes are allowed. OpenCv images have padding at the end of each line in these that are not aligned to 4 bytes
   if (iWidth*3%4!=0) {
      iWidth+=iWidth*3%4;//resize to avoid padding
      resize(iWidth, m_GlWindowSize.height);
   }
   else {
      //resize the image to the size of the GL window
      if (m_UndInputImage.rows!=0)
         cv::resize(m_UndInputImage, m_ResizedImage, m_GlWindowSize);
   }

}

// Test using ArUco to display a 3D cube in OpenCV
void ArUco::draw3DCube(cv::Mat img, int markerInd) {
   if(m_Markers.size() > markerInd) {
      aruco::CvDrawingUtils::draw3dCube(img, m_Markers[markerInd], m_CameraParams); 
   }
}

void ArUco::draw3DAxis(cv::Mat img, int markerInd) {
   if(m_Markers.size() > markerInd) {
      aruco::CvDrawingUtils::draw3dAxis(img, m_Markers[markerInd], m_CameraParams); 
   }
   
}
