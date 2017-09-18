/*********************************************************************
* Animacion-Supertoroide-3D                                          *
* Esto es un pequeño ejemplo para mostrar las prestaciones de cálcu- *
* lo de las placas STM32F407. Se ha utilizado una pantalla FT811 pa- *
* la visualización gráfica. Se muestra un Supertoroide (superficie   *
* 3D perteneciente a la familia de las Supercuadricas), en el cual   *
* se ha animado unos de los parámetros que afectan a la geometría de *
* la superficie. A continuación se calcula de forma paramétrica los  *
* vértices de la figura y la intensidad de un foco de luz que se re- *
* fleja en su superficie. Dicha intensidad se utiliza para variar el *
* color de los puntos dibujados con la FT811, proyectándolos desde el*
* espacio 3D al tamaño de vista de la pantalla a 800x480 píxeles.    *
* Basado en el código de Andy Sloane:                                *
* https://www.a1k0n.net/2011/07/20/donut-math.html                   *
*                                                                    *
* Autor: RndMnkIII (rndmnkiii@gmail.com). 2017                       *
* Ft81xTeam -> https://foro.ft81xmania.com                           *
*********************************************************************/
#define ARM_MATH_CM4
#include "arm_math.h" 
#include <math.h>
//#include <EEPROM.h> 
#include <SPI.h>
#include <GD2UB.h>

#include "ColorRGB.h"
#define WITH_FPU 1

// Enable the FPU (Cortex-M4 - STM32F4xx and higher)
// http://infocenter.arm.com/help/topic/com.arm.doc.dui0553a/BEHBJHIG.html
void enablefpu() {
  __asm volatile
  (
    "  ldr.w r0, =0xE000ED88    \n"  /* The FPU enable bits are in the CPACR. */
    "  ldr r1, [r0]             \n"  /* read CAPCR */
    "  orr r1, r1, #( 0xf << 20 )\n" /* Set bits 20-23 to enable CP10 and CP11 coprocessors */
    "  str r1, [r0]              \n" /* Write back the modified value to the CPACR */
    "  dsb                       \n" /* wait for store to complete */
    "  isb"                          /* reset pipeline now the FPU is enabled */
  );
}

const uint8_t ThetaNsegs = 24;
const uint8_t PhiNsegs = 42;
const float demoPI = 3.141593f;
const float demo2PI = 6.283185f;
const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 480;

float32_t incAnguloA=0.0f;
float32_t incAnguloB=0.0f;
float32_t tim1=0.0f;
int sube=1;

//Para almacenar coordenadas Donut 2D 24*42 vertices proyectadas en la pantalla
int16_t coord2Dx[1008];
int16_t coord2Dy[1008];
//Para almacenar la luminacia (calculo del reflejo de luz)
uint8_t lumi[1152];  

//Rotamos sobre el eje X, A radianes, y sobre el eje Z, B radianes
//devuelve el numero de vertices a dibujar, como máximo 512
uint16_t calculaToroide(float A, float B)
{
  const float theta_inc = demo2PI / ThetaNsegs;
  const float phi_inc = demo2PI / PhiNsegs;
  const float R1 = 1.0f;
  const float R2 = 2.0f;
  const float K2 = 5.0f;
  const uint16_t AnchoPanta = 800;
  const uint16_t AltoPanta = 480;
  uint16_t cuenta=0;

  //calculalos K1 basado en el tamaño de pantalla
  const float K1 = AnchoPanta*K2*3.0/(8.0*(R1+R2)); //desplazado 3/4 desde el centro hasta el lado de la pantalla

  //precalculamos valores de seno y coseno de A y B
  float cosA = arm_cos_f32(A), sinA = arm_sin_f32(A);
  float cosB = arm_cos_f32(B), sinB = arm_sin_f32(B);

  //Theta gira alrededor del circulo (corte seccional) del toroide
  for(float theta=0.0f; theta < demo2PI; theta+=theta_inc)
  {
    //precalculamos seno y coseno de theta
    float costheta = arm_cos_f32(theta), sintheta = arm_sin_f32(theta);

    //phi gira alrededor del eje de revolucion del toroide
    for(float phi=0.0f; phi < demo2PI; phi+=phi_inc)
    {
      //precalculamos seno y coseno de phi
      float cosphi = arm_cos_f32(phi), sinphi = arm_sin_f32(phi);

      //coordenadas x,y del circulo antes de rotarlo alrededor del eje de revolucion
      float circlex = R2 + R1 * costheta;
      float circley = R1 * sintheta;

      //Coordenadas 3d despues de la rotacion alrededor del eje de revolucion
      float x = circlex * (cosB*cosphi + sinA*sinB*sinphi) - circley*cosA*sinB;
      float y = circlex * (sinB*cosphi - sinA*cosB*sinphi) + circley*cosA*cosB;
      float z = K2 + cosA*circlex*sinphi + circley*sinA;
      float ooz = 1.0/z;

      //proyeccion del vertice 3d en coordenadas 2d de pantalla
      int xp = (int16_t) (AnchoPanta/2 + K1*ooz*x);
      int yp = (int16_t) (AltoPanta/2 - K1*ooz*y);

      //metodo "rapido y sucio" para calcular la luminancia
      //-sqrt(2) <= L <= sqrt(2)
      float L = cosphi*costheta*sinB - cosA*costheta*sinphi - sinA*sintheta + cosB *(cosA*sintheta - costheta*sinA*sinphi);
      //Si L < 0 la superficie apunta alejandose de nosotros y no se dibuja.
      if(L > 0.0f)
      {
        coord2Dx[cuenta]=xp;
        coord2Dy[cuenta]=yp;
        lumi[cuenta]=(uint8_t) (L * 180.5f); //L como maximo vale sqrt(2.0) -> 1.414214
        cuenta++;
      }  
    }
  } 
  return cuenta;
}

float potencia(float f, float p)
{
  int32_t signo;
  float valorAbsf;
  signo = (f<0.0)?-1:1;
  valorAbsf = (f<0.0)?-f:f;

  if(valorAbsf < 0.00001)
    return 0.0f;
  else
    return signo * powf(valorAbsf, p);
}

uint16_t calculaSuperToroide(float A, float B, float n1, float n2)
{
  const float theta_inc = demo2PI / ThetaNsegs;
  const float phi_inc = demo2PI / PhiNsegs;
  const float R1 = 1.0f;
  const float R2 = 2.0f;
  const float K2 = 5.0f;
  const uint16_t AnchoPanta = 800;
  const uint16_t AltoPanta = 480;
  uint16_t cuenta=0;

  //calculalos K1 basado en el tamaño de pantalla
  const float K1 = AnchoPanta*K2*3.0/(8.0*(R1+R2)); //desplazado 3/4 desde el centro hasta el lado de la pantalla

  //precalculamos valores de seno y coseno de A y B
  float cosA = arm_cos_f32(A), sinA = arm_sin_f32(A);
  float cosB = arm_cos_f32(B), sinB = arm_sin_f32(B);

  //Theta gira alrededor del circulo (corte seccional) del toroide
  for(float theta=0.0f; theta < demo2PI; theta+=theta_inc)
  {
    //precalculamos seno y coseno de theta
    float costheta = potencia(arm_cos_f32(theta),n1), sintheta = potencia(arm_sin_f32(theta),n1);

    //phi gira alrededor del eje de revolucion del toroide
    for(float phi=0.0f; phi < demo2PI; phi+=phi_inc)
    {
      //precalculamos seno y coseno de phi
      float cosphi = potencia(arm_cos_f32(phi),n2), sinphi = potencia(arm_sin_f32(phi),n2);

      //coordenadas x,y del circulo antes de rotarlo alrededor del eje de revolucion
      float circlex = R2 + R1 * costheta;
      float circley = R1 * sintheta;

      //Coordenadas 3d despues de la rotacion alrededor del eje de revolucion
      float x = circlex * (cosB*cosphi + sinA*sinB*sinphi) - circley*cosA*sinB;
      float y = circlex * (sinB*cosphi - sinA*cosB*sinphi) + circley*cosA*cosB;
      float z = K2 + cosA*circlex*sinphi + circley*sinA;
      float ooz = 1.0/z;

      //proyeccion del vertice 3d en coordenadas 2d de pantalla
      int xp = (int16_t) (AnchoPanta/2 + K1*ooz*x);
      int yp = (int16_t) (AltoPanta/2 - K1*ooz*y);

      //metodo "rapido y sucio" para calcular la luminancia
      //-sqrt(2) <= L <= sqrt(2)
      float L = cosphi*costheta*sinB - cosA*costheta*sinphi - sinA*sintheta + cosB *(cosA*sintheta - costheta*sinA*sinphi);
      //Si L < 0 la superficie apunta alejandose de nosotros y no se dibuja.
      if(L > 0.0f)
      {
        coord2Dx[cuenta]=xp;
        coord2Dy[cuenta]=yp;
        lumi[cuenta]=(uint8_t) (L * 180.5f); //L como maximo vale sqrt(2.0) -> 1.414214
        cuenta++;
      }  
    }
  } 
  return cuenta;
} 
void visualizaDonut(uint16_t cnt)
{ 
  GD.ClearColorRGB(0, 0, 0);
  GD.Clear();
  GD.Begin(POINTS);
  GD.PointSize(16*8);
  for(int i=0; i<cnt; i++)
  {
    GD.ColorRGB(lumi[i],lumi[i],lumi[i]);
    GD.Vertex2f((int16_t)(coord2Dx[i]*16), (int16_t)(coord2Dy[i]*16));
  } 
  GD.swap();
}

float square(float t)
{
  return t*t;
}

// 0.0 <= t <= 1.0
float ParametricBlend(float t)
{
    float sqt = square(t);
    return sqt / (2.0f * (sqt - t) + 1.0f);
}


void setup()
{
  if(WITH_FPU) {
    enablefpu();
  }
  GD.begin();
}

void loop()
{
    uint16_t nverts;
    
    //nverts=calculaToroide(incAnguloA,incAnguloB);
    //nverts=calculaSuperToroide(incAnguloA,incAnguloB, 0.8f+ParametricBlend(tim1), 1.5f);
    nverts=calculaSuperToroide(incAnguloA,incAnguloB, 1.75f, 0.7f+2*ParametricBlend(tim1));
    if(sube == 1)
      tim1+=0.005;
    else
      tim1-=0.005;

    if (tim1 > 1.0f)
    {
      sube=0;
      tim1=1.0f;   
    }
    
    if (tim1 < 0.0f) 
    {
      sube=1;
      tim1=0.0f;
    }
    //nverts=calculaToroide(angA,angB);
    visualizaDonut(nverts);
    
    incAnguloA+=0.025f;
    incAnguloB-=0.065f;
    if((incAnguloA > demo2PI) || (incAnguloA < -demo2PI)) incAnguloA=0.0f;
    if((incAnguloB > demo2PI) || (incAnguloB < -demo2PI)) incAnguloB=0.0f;
}
