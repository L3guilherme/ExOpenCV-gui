#include <QtWidgets>
#include <QtOpenGL>
#include <GL/freeglut.h>

#include "openglp.h"

#define RED 1
#define GREEN 2
#define BLUE 3
#define ORANGE 4


OpenGLp::OpenGLp(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
    xRot = 0;
    yRot = 0;
    zRot = 0;
}

OpenGLp::~OpenGLp()
{

}

QSize OpenGLp::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize OpenGLp::sizeHint() const
{
    return QSize(400, 400);
}

static void qNormalizeAngle(int &angle)
{
    while (angle < 0)
        angle += 360 * 16;
    while (angle > 360)
        angle -= 360 * 16;
}

void OpenGLp::setXRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != xRot) {
        xRot = angle;
        emit xRotationChanged(angle);
        updateGL();
    }
}

void OpenGLp::setYRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != yRot) {
        yRot = angle;
        emit yRotationChanged(angle);
        updateGL();
    }
}

void OpenGLp::setZRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != zRot) {
        zRot = angle;
        emit zRotationChanged(angle);
        updateGL();
    }
}


void OpenGLp::initializeGL()
{

    int argc = 0;
    char *argv[1] = {(char*)""};
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);


    qglClearColor(Qt::white);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_SMOOTH);

    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    // Set lighting intensity and color
    GLfloat qaAmbientLight[]	= {0.3, 0.3, 0.3, 1.0};
    GLfloat qaDiffuseLight[]	= {0.6, 0.6, 0.6, 1.0};
    GLfloat qaSpecularLight[]	= {1.0, 1.0, 1.0, 1.0};
    glLightfv(GL_LIGHT0, GL_AMBIENT, qaAmbientLight);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, qaDiffuseLight);
    glLightfv(GL_LIGHT0, GL_SPECULAR, qaSpecularLight);

    // Set the light position
    GLfloat qaLightPosition[]	= {0.f, 0.f, 10.f, 1.0};
    glLightfv(GL_LIGHT0, GL_POSITION, qaLightPosition);
}

void OpenGLp::resizeGL(int width, int height)
{
    int side = qMin(width, height);
    glViewport((width - side) / 2, (height - side) / 2, side, side);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
#ifdef QT_OPENGL_ES_1
    glOrthof(-2, +2, -2, +2, 1.0, 15.0);
#else
    glOrtho(-25, +25, -25, +25, -20.0, 20.0);
#endif
    glMatrixMode(GL_MODELVIEW);
}

void OpenGLp::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
}

void OpenGLp::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - lastPos.x();
    int dy = event->y() - lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        setXRotation(xRot + 8 * dy);
        setYRotation(yRot + 8 * dx);
    } else if (event->buttons() & Qt::RightButton) {
        setXRotation(xRot + 8 * dy);
        setZRotation(zRot + 8 * dx);
    }

    lastPos = event->pos();
}

void OpenGLp::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glTranslatef(0.0, 0.0, 0.0);
    glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
    glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
    glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);

    // Set the camera
    gluLookAt(	0.f, 0.0f, 5.f,
                0.f, 0.0f,  0.f,
                0.0f, 1.0f,  0.0f);

    GLfloat qaCinza[] = {0.9f, 0.9f, 0.9f, 1.0};
    GLfloat qaR[] = {1.f, 0.f, 0.f, 1.f};
    GLfloat qaG[] = {0.f, 1.f, 0.f, 1.f};
    GLfloat qaB[] = {0.f, 0.f, 1.f, 1.f};
    glMaterialfv(GL_FRONT, GL_AMBIENT, qaCinza);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, qaCinza);
    glMaterialfv(GL_FRONT, GL_SPECULAR, qaCinza);
    glMaterialf(GL_FRONT, GL_SHININESS, 60.0);
    // Draw ground
    glColor3f(0.9f, 0.9f, 0.9f);
    glBegin(GL_QUADS);
    glVertex3f(-100.0f, 0.0f, -100.0f);
    glVertex3f(-100.0f, 0.0f,  100.0f);
    glVertex3f( 100.0f, 0.0f,  100.0f);
    glVertex3f( 100.0f, 0.0f, -100.0f);
    glEnd();

    glMaterialfv(GL_FRONT, GL_AMBIENT, qaR);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, qaR);
    glMaterialfv(GL_FRONT, GL_SPECULAR, qaR);
    glMaterialf(GL_FRONT, GL_SHININESS, 60.0);

    glColor3f(1.0f, 0.0f, 0.0f);

    glBegin(GL_LINES);
    glVertex3f( 0.0f, 0.0f, 0.0f);
    glVertex3f( 10.0f, 0.0f,0.0f);
    glEnd();

    glMaterialfv(GL_FRONT, GL_AMBIENT, qaG);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, qaG);
    glMaterialfv(GL_FRONT, GL_SPECULAR, qaG);
    glMaterialf(GL_FRONT, GL_SHININESS, 60.0);

    glColor3f(0.0f, 1.0f, 0.0f);

    glBegin(GL_LINES);
    glVertex3f( 0.0f, 0.0f, 0.0f);
    glVertex3f( 0.0f, 10.0f,0.0f);
    glEnd();

    glMaterialfv(GL_FRONT, GL_AMBIENT, qaB);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, qaB);
    glMaterialfv(GL_FRONT, GL_SPECULAR, qaB);
    glMaterialf(GL_FRONT, GL_SHININESS, 60.0);

    glColor3f(0.0f, 0.0f, 1.0f);

    glBegin(GL_LINES);
    glVertex3f( 0.0f, 0.0f, 0.0f);
    glVertex3f( 0.0f, 0.0f,10.0f);
    glEnd();

    draw();

    glPopMatrix();
}

void OpenGLp::draw()
{
    //    glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads
    //          // Top face (y = 1.0f)
    //          // Define vertices in counter-clockwise (CCW) order with normal pointing out
    //          glColor3f(0.0f, 1.0f, 0.0f);     // Green
    //          glVertex3f( 1.0f, 1.0f, -1.0f);
    //          glVertex3f(-1.0f, 1.0f, -1.0f);
    //          glVertex3f(-1.0f, 1.0f,  1.0f);
    //          glVertex3f( 1.0f, 1.0f,  1.0f);

    //          // Bottom face (y = -1.0f)
    //          glColor3f(1.0f, 0.5f, 0.0f);     // Orange
    //          glVertex3f( 1.0f, -1.0f,  1.0f);
    //          glVertex3f(-1.0f, -1.0f,  1.0f);
    //          glVertex3f(-1.0f, -1.0f, -1.0f);
    //          glVertex3f( 1.0f, -1.0f, -1.0f);

    //          // Front face  (z = 1.0f)
    //          glColor3f(1.0f, 0.0f, 0.0f);     // Red
    //          glVertex3f( 1.0f,  1.0f, 1.0f);
    //          glVertex3f(-1.0f,  1.0f, 1.0f);
    //          glVertex3f(-1.0f, -1.0f, 1.0f);
    //          glVertex3f( 1.0f, -1.0f, 1.0f);

    //          // Back face (z = -1.0f)
    //          glColor3f(1.0f, 1.0f, 0.0f);     // Yellow
    //          glVertex3f( 1.0f, -1.0f, -1.0f);
    //          glVertex3f(-1.0f, -1.0f, -1.0f);
    //          glVertex3f(-1.0f,  1.0f, -1.0f);
    //          glVertex3f( 1.0f,  1.0f, -1.0f);

    //          // Left face (x = -1.0f)
    //          glColor3f(0.0f, 0.0f, 1.0f);     // Blue
    //          glVertex3f(-1.0f,  1.0f,  1.0f);
    //          glVertex3f(-1.0f,  1.0f, -1.0f);
    //          glVertex3f(-1.0f, -1.0f, -1.0f);
    //          glVertex3f(-1.0f, -1.0f,  1.0f);

    //          // Right face (x = 1.0f)
    //          glColor3f(1.0f, 0.0f, 1.0f);     // Magenta
    //          glVertex3f(1.0f,  1.0f, -1.0f);
    //          glVertex3f(1.0f,  1.0f,  1.0f);
    //          glVertex3f(1.0f, -1.0f,  1.0f);
    //          glVertex3f(1.0f, -1.0f, -1.0f);
    //       glEnd();  // End of drawing color-cube

    //       glutSolidTorus (0.275, 0.85, 8, 15);


        // Set material properties
        GLfloat qaBlack[] = {0.0, 0.0, 0.0, 1.0};
        GLfloat qaLar[] = {1.0f, 0.5f , 0.5f};
        GLfloat qaWhite[] = {1.0, 1.0, 1.0, 1.0};
//        glMaterialfv(GL_FRONT, GL_AMBIENT, qaWhite);
//        glMaterialfv(GL_FRONT, GL_DIFFUSE, qaWhite);
//        glMaterialfv(GL_FRONT, GL_SPECULAR, qaWhite);
//        glMaterialf(GL_FRONT, GL_SHININESS, 60.0);
//    glColor3f(1.0f, 1.0f, 1.0f);

//    // Draw Body
//    glTranslatef(0.0f ,0.75f, 0.0f);
//    glutSolidSphere(0.75f,20,20);

//    // Draw Head
//    glTranslatef(0.0f, 1.0f, 0.0f);
//    glutSolidSphere(0.25f,20,20);

    // Draw Eyes

    glMaterialfv(GL_FRONT, GL_AMBIENT, qaBlack);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, qaBlack);
    glMaterialfv(GL_FRONT, GL_SPECULAR, qaWhite);
    glMaterialf(GL_FRONT, GL_SHININESS, 60.0);

    glBegin(GL_LINES);
    glVertex3f( 0.0f, 0.0f, 0.0f);
    glVertex3f( 0.0f, 5.0f,0.0f);

    glVertex3f( 0.0f, 5.0f,0.0f);
    glVertex3f( 5.0f, 7.0f, 0.0f);

    glVertex3f( 5.0f, 7.0f, 0.0f);
    glVertex3f( 10.0f, 5.0f, 0.0f);

    glVertex3f( 10.0f, 5.0f, 0.0f);
    glVertex3f( 15.0f, 5.0f,0.0f);

    glVertex3f( 15.0f, 5.0f,0.0f);
    glVertex3f( 15.0f, 0.0f, 0.0f);

    glVertex3f( 15.0f, 0.0f, 0.0f);
    glVertex3f( 0.0f,  0.0f,0.0f);


    glEnd();



//    glPushMatrix();
//    glColor3f(0.0f,0.0f,0.0f);
//    glTranslatef(0.05f, 0.10f, 0.18f);
//    glutSolidSphere(0.05f,10,10);
//    glTranslatef(-0.1f, 0.0f, 0.0f);
//    glutSolidSphere(0.05f,10,10);
//    glPopMatrix();

//    glMaterialfv(GL_FRONT, GL_AMBIENT, qaLar);
//    glMaterialfv(GL_FRONT, GL_DIFFUSE, qaLar);
//    glMaterialfv(GL_FRONT, GL_SPECULAR, qaWhite);
//    glMaterialf(GL_FRONT, GL_SHININESS, 60.0);

//    // Draw Nose
//    glColor3f(1.0f, 0.5f , 0.5f);
//    glutSolidCone(0.08f,0.5f,10,2);
}
