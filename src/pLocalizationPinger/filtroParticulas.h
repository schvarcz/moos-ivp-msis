#ifndef FILTROPARTICULAS_H
#define FILTROPARTICULAS_H

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <time.h>
#include "configuracaoes.h"
#include "landmark.h"
#include "particula.h"

using namespace std;

class FiltroParticulas
{
public:
    FiltroParticulas();
    void executarFiltro(vector <xyz> poseXYZ, vector <xyz> yawPitchRoll,vector <xyz> velXYZ,vector <vector<transponder> > transponders,vector <Landmark> landmarksUsados, int idExec);
    void roleta( vector <Particula> & populacao);
    void criarPopulacao(IntervalVector searchSpace, vector <Particula> & populacao, xyz yawPitchRoll);
};

#endif // FILTROPARTICULAS_H
