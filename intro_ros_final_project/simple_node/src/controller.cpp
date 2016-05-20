/* This file containts the function controller */
float controller(float error, float *integral, float *errora, float fkp, float fkd, float fki, float tsample)
{
  float velocitat;
  *integral += *integral*tsample; // integracio
  velocitat=-fkp*(error)-fki*(*integral)-fkd*(error-*errora)/tsample;
  *errora = error;

  return velocitat;
}
