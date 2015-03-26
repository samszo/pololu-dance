/* Project create by abdellah and zayd
   pololu m3pi

*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>
#include <unistd.h>
#include <fcntl.h>  
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc_c.h>

#define MIN3(a, b, c) ( (a) < (b) ? ( (a) < (c) ? (a) : ( (b) < (c) ? (b) : (c) ) ) : ( (b) < (c) ? (b) : (c) ) )
#define MAX3(a, b, c) ( (a) > (b) ? ( (a) > (c) ? (a) : ( (b) > (c) ? (b) : (c) ) ) : ( (b) > (c) ? (b) : (c) ) )

typedef struct fvec4_t fvec4_t;
struct fvec4_t {
  float * data;
  int nelem;
  int curelem;
  int maxelem;
};

static fvec4_t * new_fvec4(int nelem) {
  fvec4_t * v = malloc(sizeof *v);
  assert(v);
  v->nelem = nelem;
  v->curelem = v->maxelem = 0;
  v->data = malloc(4 * v->nelem * sizeof *v->data);
  assert(v->data);
  return v;
}
double calculerDistance(int ax, int ay,int bx,int by){
 
    return sqrt((double)((bx - ax)*(bx - ax)) + ((by - ay)*(by - ay)));
 
}
static void free_fvec4(fvec4_t * v) {
  if(v) {
    if(v->data)
      free(v->data);
    free(v);
  }
}

static void fill_fvec4(fvec4_t * v, float *vec3) {
  int i;
  if(v->curelem == v->nelem) {
    v->curelem = 0;
  }
  for(i = 0; i < 3; i++)
    v->data[4 * v->curelem + i] = vec3[i];
  /* ou memcpy(&(v->data[4 * v->curelem]), vec3, 3 * sizeof *v->data);  */
  v->data[4 * v->curelem + 3] = 1;
  if(v->maxelem < ++v->curelem)
    v->maxelem = v->curelem;
}

int compar(const void * p0, const void * p1) {
  const float * f0 = p0, * f1 = p1;
  return f0[3] < f1[3] ? 1 : -1;
}

static void clusterize_fvec4(fvec4_t * v, fvec4_t * cv, float thresholdd) {
  int ii, i, j, nearesti;
  float nearestd, d, dx ,dy, dr;
  cv->curelem = cv->maxelem = cv->maxelem >= 2 ? 2 : 0;
  for(ii = 0; ii < cv->maxelem; ii++)
    cv->data[4 * ii + 3] = 0.0;
  for(ii = 0; ii < v->maxelem; ii++) {
    i = (v->curelem + ii) % v->nelem;
    //i = (v->curelem - 1 - ii + v->nelem) % v->nelem;
    for(j = 0, nearestd = FLT_MAX, nearesti = -1; j < cv->maxelem; j++) {
      dx = v->data[4 * i] - cv->data[4 * j];
      dy = v->data[4 * i + 1] - cv->data[4 * j + 1];
      dr = v->data[4 * i + 2] - cv->data[4 * j + 2];
      d = sqrt(dx * dx + dy * dy + dr * dr);
      if(nearestd > d) {
	nearestd = d;
	nearesti = j;
      }
    }
    if(nearestd < thresholdd) {
      for(j = 0; j < 3; j++)
	cv->data[4 * nearesti + j] = (v->data[4 * i + j] + cv->data[4 * nearesti + j]) / 2.0;
      cv->data[4 * nearesti + 3] += thresholdd / (nearestd + 0.1);
    } else {
      memcpy(&(cv->data[4 * cv->curelem]), &(v->data[4 * i]), 4 * sizeof *cv->data);
      cv->maxelem = ++cv->curelem;
      if(cv->maxelem >= cv->nelem)
	break;
    }
  }
  qsort(cv->data, cv->nelem, 4 * sizeof *cv->data, compar);
}

static void rgb2hsv(float r, float g, float b, float * h, float * s, float * v) {
  float min = MIN3(r, g, b), max = MAX3(r, g, b);
  *v = max;
  *s = (max != 0.0) ? ((max - min) / max) : 0.0;
  if(*s == 0.0)
    *h = 1.0;
  else {
    float delta = 1.0 / (max - min);
    if(r == max)
      *h = delta * (g - b) ;
    else if(g == max)
      *h = 2.0 + delta * (b - r);
    else if(b == max)
      *h = 4.0 + delta * (r - g);
    *h = (*h < 0.0) ? (*h / 6.0 + 1.0) : (*h / 6.0);
  }
}
static void garderLeVert(IplImage * img) {
  int i, j;
  unsigned char * dd = (unsigned char *)img->imageData;
  float r, g, b, h, s, v;
  assert(img->nChannels == 3);
  for(i = 0; i < img->height; i++) {
    for(j = 0; j < img->width; j++) {
      r = dd[i * img->widthStep + j * img->nChannels + 0] / 255.0;
      g = dd[i * img->widthStep + j * img->nChannels + 1] / 255.0;
      b = dd[i * img->widthStep + j * img->nChannels + 2] / 255.0;
      rgb2hsv(r, g, b, &h, &s, &v);
      if(h >= 0.1 && h <= 0.5 )
	dd[i * img->widthStep + j * img->nChannels + 0] = dd[i * img->widthStep + j * img->nChannels + 1] = dd[i * img->widthStep + j * img->nChannels + 2] = s * 255.0;
      else
	dd[i * img->widthStep + j * img->nChannels + 0] = dd[i * img->widthStep + j * img->nChannels + 1] = dd[i * img->widthStep + j * img->nChannels + 2] = 0;
    }
  }
}

static void garderLeBleu(IplImage * img) {
  int i, j;
  unsigned char * dd = (unsigned char *)img->imageData;
  float r, g, b, h, s, v;
  assert(img->nChannels == 3);
  for(i = 0; i < img->height; i++) {
    for(j = 0; j < img->width; j++) {
      b = dd[i * img->widthStep + j * img->nChannels + 0] / 255.0;
      g = dd[i * img->widthStep + j * img->nChannels + 1] / 255.0;
      r = dd[i * img->widthStep + j * img->nChannels + 2] / 255.0;
      rgb2hsv(r, g, b, &h, &s, &v);
 if(h >= 0.4 && h <= 0.7 && v >= 0.5 && v <= 0.9 &&  s >= 0.2 && s <= 1)
	dd[i * img->widthStep + j * img->nChannels + 0] = dd[i * img->widthStep + j * img->nChannels + 1] = dd[i * img->widthStep + j * img->nChannels + 2] = 255.0;
      else
	dd[i * img->widthStep + j * img->nChannels + 0] = dd[i * img->widthStep + j * img->nChannels + 1] = dd[i * img->widthStep + j * img->nChannels + 2] = 0;
    }
  }
}

static void garderLeRouge(IplImage * img) {
 int i, j;
  unsigned char * dd = (unsigned char *)img->imageData;
  float r, g, b, h, s, v;
  assert(img->nChannels == 3);
  for(i = 0; i < img->height; i++) {
    for(j = 0; j < img->width; j++) {
      b = dd[i * img->widthStep + j * img->nChannels + 0] / 255.0;
      g = dd[i * img->widthStep + j * img->nChannels + 1] / 255.0;
      r = dd[i * img->widthStep + j * img->nChannels + 2] / 255.0;
      rgb2hsv(r, g, b, &h, &s, &v);
   if(h >= 0.7 && h <= 1 &&  v >= 0.6 && v <= 1 &&  s >= 0.3 && s <= 1)
	dd[i * img->widthStep + j * img->nChannels + 0] = dd[i * img->widthStep + j * img->nChannels + 1] = dd[i * img->widthStep + j * img->nChannels + 2] =  255.0;
      else
	dd[i * img->widthStep + j * img->nChannels + 0] = dd[i * img->widthStep + j * img->nChannels + 1] = dd[i * img->widthStep + j * img->nChannels + 2] = 0;
    }
  }
}
// fonction qui calcule l'angle
static double angle( double a, double b, double c )
{

 return  acos((pow(a, 2) + pow(b, 2) - pow(c, 2)) / (2 * a*b))*180/M_PI; 
}
int main(int argc, char ** argv) {
char trans[]="";
  IplImage* img = NULL, * imgG = NULL,* ori = NULL,* imgb = NULL,* imgr = NULL,* imgball = NULL;
  CvCapture * capture = NULL;
  CvSize s;
int xv1=0,yv1=0,xv2=0,yv2=0,xb=0,yb=0,xr=0,yr=0,xball=0,yball=0;
  int i, j;
  CvMemStorage * memSto = cvCreateMemStorage(0);
  CvSeq * res = NULL;
  fvec4_t * v = new_fvec4(100), * cv = new_fvec4(100);
fvec4_t * r = new_fvec4(100), * cr= new_fvec4(100);
  fvec4_t * b = new_fvec4(100), * cb = new_fvec4(100);
  fvec4_t  * cball = new_fvec4(100);
  if(!(capture = cvCaptureFromCAM(1)))
    capture = cvCaptureFromCAM(CV_CAP_ANY);
  cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640);
  cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480);
  s.width = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
  s.height = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);
  fprintf(stderr, "dimensions du flux %dx%d\n", s.width, s.height);
  cvNamedWindow("Intro OpenCV", CV_WINDOW_NORMAL);
  cvNamedWindow("vert", CV_WINDOW_NORMAL);
  cvNamedWindow("ball", CV_WINDOW_NORMAL);
  cvNamedWindow("blue", CV_WINDOW_NORMAL);
  cvNamedWindow("rouge", CV_WINDOW_NORMAL);
  imgG = cvCreateImage(s, IPL_DEPTH_8U, 1);
  while((img = cvQueryFrame(capture))) {

    cvSmooth(img, img, CV_GAUSSIAN, 5, 5, 0, 0);
ori=cvCloneImage(img);
imgb=cvCloneImage(img);
   imgr=cvCloneImage(img);
imgball=cvCloneImage(img);
 garderLeVert(img);
    cvCvtColor(img, imgG, CV_RGB2GRAY);
    res = cvHoughCircles(imgG, memSto, CV_HOUGH_GRADIENT, 2, imgG->width / 4, 80, 20, 2, 20);
    for(i = 0; i < res->total; i++) {
      float * p = (float *)cvGetSeqElem(res, i);
      CvPoint pt = cvPoint(cvRound(p[0]), cvRound(p[1]));
      cvCircle(img, pt, cvRound(p[2]), CV_RGB(0xff, 0, 0), 1, 8, 0);
      fill_fvec4(v, p);
      clusterize_fvec4(v, cv, 10);
      if(cv->maxelem >= 2) {
	for(j = 0; j < 2; j++) {
	  pt = cvPoint(cvRound(cv->data[4 * j]), cvRound(cv->data[4 * j + 1]));

if(j==0){
xv1=pt.x;
yv1=pt.y;

}
else
{
xv2=pt.x;
yv2=pt.y;

}

	  cvCircle(img, pt, cvRound(MIN(100.0, fabs(cv->data[4 * j + 2]))), CV_RGB(0, 0xff, 0), 1, 8, 0);
 cvCircle(ori, pt, cvRound(MIN(100.0, fabs(cv->data[4 * j + 2]))), CV_RGB(0, 0, 0xff), 1, 8, 0);
	}
      }
    }
 garderLeBleu(imgb);
 cvCvtColor(imgb, imgG, CV_RGB2GRAY);
    res = cvHoughCircles(imgG, memSto, CV_HOUGH_GRADIENT, 2, imgG->width / 4, 80, 20, 2, 20);
    for(i = 0; i < res->total; i++) {
      float * p = (float *)cvGetSeqElem(res, i);
      CvPoint pt = cvPoint(cvRound(p[0]), cvRound(p[1]));
      cvCircle(imgb, pt, cvRound(p[2]), CV_RGB(0xff, 0, 0), 1, 8, 0);
      fill_fvec4(b, p);
      clusterize_fvec4(b, cb, 10);
      if(cb->maxelem >= 2) {
	for(j = 0; j < 1; j++) {
	  pt = cvPoint(cvRound(cb->data[4 * j]), cvRound(cb->data[4 * j + 1]));
xb=pt.x;
yb=pt.y;

	  cvCircle(imgb, pt, cvRound(MIN(100.0, fabs(cb->data[4 * j + 2]))), CV_RGB(0, 0, 0xff), 1, 8, 0);
 cvCircle(ori, pt, cvRound(MIN(100.0, fabs(cb->data[4 * j + 2]))), CV_RGB(0,0, 0xff), 1, 8, 0);
	}
      }
    }
garderLeRouge(imgr);
 cvCvtColor(imgr, imgG, CV_RGB2GRAY);
    res = cvHoughCircles(imgG, memSto, CV_HOUGH_GRADIENT, 2, imgG->width / 4, 80, 20, 2, 10);

    for(i = 0; i < res->total; i++) {
      float * p = (float *)cvGetSeqElem(res, i);
      CvPoint pt = cvPoint(cvRound(p[0]), cvRound(p[1]));
      cvCircle(imgr, pt, cvRound(p[2]), CV_RGB(0xff, 0, 0), 1, 8, 0);
      fill_fvec4(r, p);
      clusterize_fvec4(r, cr, 10);
      if(cr->maxelem >= 2) {
	for(j = 0; j < 1; j++) {
	  pt = cvPoint(cvRound(cr->data[4 * j]), cvRound(cr->data[4 * j + 1]));
xr=pt.x;
yr=pt.y;


	  cvCircle(imgr, pt, cvRound(MIN(100.0, fabs(cr->data[4 * j + 2]))), CV_RGB(0, 0, 0xff), 1, 8, 0);
 cvCircle(ori, pt, cvRound(MIN(100.0, fabs(cr->data[4 * j + 2]))), CV_RGB(0,0, 0xff), 1, 8, 0);
	}

      }
    }
// on dessine une ligne entre le centre du robot et la balle
CvPoint  pt1= cvPoint(cvRound(xb), cvRound(yb));
CvPoint pt2= cvPoint(cvRound(xball), cvRound(yball));
 cvLine(ori,  pt1,  pt2,  CV_RGB(0, 0, 0xff),1, 8, 0 );


double p1=calculerDistance(xv1,yv1,xr,yr);
double p2=calculerDistance(xv2,yv2,xr,yr);
CvPoint pte;
if(p1< p2)// pr detecter vert qui est en avant 

	  pte = cvPoint(cvRound(xv1), cvRound(yv1));
else
	  pte = cvPoint(cvRound(xv2), cvRound(yv2));
// pr detecter ball on augmente la taille des cercle que le robot doit detecter
  garderLeRouge(imgball);
 cvCvtColor(imgball, imgG, CV_RGB2GRAY);
    res = cvHoughCircles(imgG, memSto, CV_HOUGH_GRADIENT, 2, imgG->width / 4, 80, 40, 10, 40);
    for(i = 0; i < res->total; i++) {
      float * p = (float *)cvGetSeqElem(res, i);
      CvPoint pt = cvPoint(cvRound(p[0]), cvRound(p[1]));
      cvCircle(imgball, pt, cvRound(p[2]), CV_RGB(0xff, 0, 0), 1, 8, 0);
xball=p[0];
yball=p[1];

	  cvCircle(imgball, pt, cvRound(MIN(100.0, fabs(cball->data[4 * j + 2]))), CV_RGB(0, 0, 0xff), 1, 8, 0);
 cvCircle(ori, pt, cvRound(MIN(100.0, fabs(cball->data[4 * j + 2]))), CV_RGB(0,0, 0xff), 1, 8, 0);
    }
//tourner robot:
double anglee=0;
double bball=0;

if(xball!=0 && xb !=0 && yball!=0 && yb!=0 && pte.x!=0 && pte.y!=0)
{

 bball=calculerDistance(xball,yball,xb,yb);
double bvert=calculerDistance(pte.x,pte.y,xb,yb);
double vball=calculerDistance(xball,yball,pte.x,pte.y
);
 anglee= angle(bvert, bball, vball);
printf(" l'angle est %f \n",anglee);
}
// droite ou gauche
double rball=calculerDistance(xball,yball,xr,yr);

if(anglee <10 && anglee>0)
{trans[0]='A';
printf("avance \n");}
else if(anglee>0)
{

if(rball<bball )
{
//gauche
trans[0]='G';
printf("gauche \n");
}
else
{
//droite
trans[0]='D';
printf("droite \n");

}
}
if(bball<60) break;
printf(" la distance:%f",bball);
  int fd = open("/dev/ttyACM0", O_RDWR);
 if (fd == -1){
	fprintf(stderr, "Unable to open the device\n");
	exit(0);
  }

  write(fd, trans, sizeof trans);

  }
    cvShowImage("Intro OpenCV", ori);
  cvShowImage("vert", img);
 cvShowImage("blue", imgb);
cvShowImage("rouge", imgr);
cvShowImage("ball", imgball);
  
    if( (cvWaitKey(10) & 0xFF) == 27 )
      break;
  }
trans[0]='M';
 int fd = open("/dev/ttyACM0", O_RDWR);
 write(fd, trans, sizeof trans);
  if(capture)
    cvReleaseCapture(&capture);
  free_fvec4(v);
  free_fvec4(cv);
  return 0;
}
