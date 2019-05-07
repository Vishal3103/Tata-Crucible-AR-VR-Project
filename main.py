from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import cv2
from PIL import Image
import numpy as np
from webcam import Webcam
from objloader import *
from constants import *
from feature_matching import *
from candidcap import *
 
class OpenGLGlyphs:

    INVERSE_MATRIX = np.array([[ 1.0, 1.0, 1.0, 1.0],
                               [-1.0,-1.0,-1.0,-1.0],
                               [-1.0,-1.0,-1.0,-1.0],
                               [ 1.0, 1.0, 1.0, 1.0]])
 
    def __init__(self):
        self.webcam = Webcam()
        self.webcam.start()
        self.cone = None
        self.sphere = None
        self.shape= None
        self.texture_background = None
 
    def _init_gl(self, Width, Height):
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(33.7, 1.3, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)         
        self.cone = OBJ('Heart.obj')
        self.sphere = OBJ('Heart_cut.obj')
        self.shape = self.cone

        glEnable(GL_TEXTURE_2D)
        self.texture_background = glGenTextures(1)
 
    def _draw_scene(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        image = self.webcam.get_current_frame()

        bg_image = cv2.flip(image, 0)
        bg_image = Image.fromarray(bg_image)     
        ix = bg_image.size[0]
        iy = bg_image.size[1]
        bg_image = bg_image.tostring("raw", "BGRX", 0, -1)

        glBindTexture(GL_TEXTURE_2D, self.texture_background)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, bg_image)

        glBindTexture(GL_TEXTURE_2D, self.texture_background)
        glPushMatrix()
        glTranslatef(0.0,0.0,-10.0)
        self._draw_background()
        glPopMatrix()
        image = self._handle_glyphs(image)
        glutSwapBuffers()

    def _handle_glyphs(self, image):
        cx, cy = findcontours(image)
        try:
            dst, rvecs, tvecs, image, boolean = detect(image)
            if boolean :
                print dst[0][0][0], dst[0][0][1]#, dst[0][2][0], dst[0][2][1]
                if cx > dst[0][0][0] and cy > dst[0][0][1] :
                    self.shape = self.sphere
                    
                    print "YES"
                else:
                    self.shape=self.cone
                rmtx = cv2.Rodrigues(rvecs)[0]

                view_matrix = np.array([[rmtx[0][0],rmtx[0][1],rmtx[0][2],tvecs[0]],
                                        [rmtx[1][0],rmtx[1][1],rmtx[1][2],tvecs[1]],
                                        [rmtx[2][0],rmtx[2][1],rmtx[2][2],tvecs[2]],
                                        [0.0       ,0.0       ,0.0       ,1.0    ]])

                view_matrix = view_matrix * self.INVERSE_MATRIX

                view_matrix = np.transpose(view_matrix)
                # load view matrix and draw shape
                glPushMatrix()
                glLoadMatrixd(view_matrix)
                glCallList(self.shape.gl_list)
                glPopMatrix()
        except:
            pass
     
    def _draw_background(self):

        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 1.0); glVertex3f(-4.0, -3.0, 0.0)
        glTexCoord2f(1.0, 1.0); glVertex3f( 4.0, -3.0, 0.0)
        glTexCoord2f(1.0, 0.0); glVertex3f( 4.0,  3.0, 0.0)
        glTexCoord2f(0.0, 0.0); glVertex3f(-4.0,  3.0, 0.0)
        glEnd( )
 
    def main(self):
        glutInit()
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
        glutInitWindowSize(640, 480)
        glutInitWindowPosition(800, 400)
        self.window_id = glutCreateWindow("nichi Technologies")
        glutDisplayFunc(self._draw_scene)
        glutIdleFunc(self._draw_scene)
        self._init_gl(640, 480)
        glutMainLoop()
        
openGLGlyphs = OpenGLGlyphs()
openGLGlyphs.main()
