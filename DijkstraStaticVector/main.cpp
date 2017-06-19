#define INFINITO 100000000
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include <iostream>
#include "Lista.h"
#include <time.h>

using std::cout;
using std::endl;
using std::ostream;
using autoreferencia::Lista;

//Declaração de função global
void Menu();

// Manipulação de arquivos
FILE* abreArquivo(char modo, char caminho[30])
{
    FILE *arquivo;
    switch(modo)
    {
        case 'g':
            arquivo = fopen(caminho, "wt");
            break;
        case 'l':
            arquivo = fopen(caminho, "rt");
            break;
        case 'a':
            arquivo = fopen(caminho, "a");
            break;
    }
    if(arquivo == NULL)
	{
        printf("Nao foi possivel abrir o arquivo.");
        Menu();
	}
	return arquivo;
}

void fechaArquivo(FILE* arquivo)
{
    fclose(arquivo);
}

class FPNaoOrdenado	{

	class Elemento{ //elemento da fila
		int vertice;
		int peso;

		public:
		Elemento(){

		}
  		Elemento  ( int v , int p )  {
			this->vertice  =  v ;
			this->peso = p ;
		}
		int _peso  ( )  {
			return  this->peso ;
		}
		int _vertice  ( )  {
	 		return  this->vertice ;
		}
		void atribuiPeso(int peso){
			this->peso = peso;
		}
	};

	public:
	int tamanho; //tamanho da fila
	Elemento vet[INFINITO]; //vetor  de tamanho mamximo definido; contendo um Elemeno(vertice,peso)

	FPNaoOrdenado (){//construtor apenas inicializa o tamanho para zero
		this->tamanho = 0;
	}

	void insere(int dado, int distancia){
		if (this->tamanho == INFINITO)
			throw logic_error ("Erro: Fila cheia");
		else{
			Elemento e =  Elemento (dado,distancia);
			this->vet[tamanho] = e;
			this->tamanho++;
		}
	}

	Elemento remove(){  //remove o elemento de maior distancia = menor peso
		int menorP, posMenorP;
	  	if (this->tamanho < 1)
			throw logic_error ("Erro: fila vazia");
  		Elemento elementoMin, temp; //
		temp = (Elemento)this->vet[0]; //seta o primeiro elemento como o menor peso
		menorP = temp._peso();
		posMenorP = 0;

		elementoMin = temp;
		for(int i= 1; i< this->tamanho; i++){
	    	temp = this->vet[i]; //
    		int pesoAtual =temp._peso();
	    	if(menorP > pesoAtual){ //se o peso do 0, for maior que o peso da posicao 1, seta o menor para o da posicao 1;
	      		menorP = pesoAtual;
	      		elementoMin = temp;
	      		posMenorP = i;
	    	}
	    	//printf(" menorP= %d \n",  menorP);
	  	}
	  	for (int i=posMenorP; i<this->tamanho-1; i++ ){//retira o elementoMin, movendo os proximos pra posicao dele
	  		this->vet[i]= this->vet[i+1]; //

		}
		this->tamanho--;
		return elementoMin;
	}

	void diminuiChave(int dado, int distancia){
		Elemento temp;
		if (this->tamanho < 1)
			throw logic_error ("Erro: fila vazia");
		temp = this->vet[0];
		if (distancia < 0)
	    	throw logic_error ("Erro: nova chave com valor incorreto");
		for(int i= 0; i< this->tamanho; i++){
	    	temp = this->vet[i]; //
	    	if(dado == temp._vertice()){
				if(distancia < this->vet[i]._peso())
	      			this->vet[i].atribuiPeso(distancia);
	    	}
	  	}
	}

	bool vazio () {
		return this->tamanho == 0;
	}

};

class Grafo  {
	public:
	class Aresta  {
		int v1 ,  v2 ,  peso;
	  	public:
		Aresta  ( int v1 , int v2 , int peso )  {
			this->v1  =  v1 ;
			this->v2  =  v2 ;
			this->peso = peso ;
		}

		int _peso  ( )  {
			return  this->peso ;
		}
		int _v1  ( )  {
	 		return  this->v1 ;
		}
		int _v2  ( )  {
	 		return  this->v2 ;
		}
	};

	class Celula {
	    friend class Grafo;
	    friend ostream& operator<< (ostream& out, const Celula& celula) {
	      out << "vertice:" << celula.vertice << endl;
	      out << "peso:"    << celula.peso    << endl;
	      return out;
	    }
		int vertice, peso;
		Celula *prox;
		Celula (int v, int p) {
		    	this->vertice = v;
				this->peso = p;
		}
		public:
		Celula (const Celula& cel) { *this = cel; }
	      bool operator== (const Celula& celula) const {
	        return this->vertice == celula.vertice;
	      }
	      bool operator!= (const Celula& celula) const {
	        return this->vertice != celula.vertice;
	      }
	      const Celula& operator= (const Celula& cel) {
	        this->vertice = cel.vertice;
	        this->peso = cel.peso;
	        return *this; // permite atribuições encadeadas
	      }
	      ~Celula () {}
	};

	Lista<Celula> *adj;
    int numVertices;
    public:
    Grafo (int numVertices) {
  		this->adj = new Lista<Celula>[numVertices];
  		this->numVertices = numVertices;
	}

	void insereAresta (int v1, int v2, int peso) {
        Celula item(v2, peso);
        this->adj[v1].insere(item);
  	}

	bool existeAresta (int v1, int v2) const {
	    Celula item(v2, 0);
	    return (this->adj[v1].pesquisa(item) != NULL);
	}

	bool listaAdjVazia (int v) {
		return this->adj[v].vazia();
	}

	Aresta *primeiroListaAdj (int v) {
	    // Retorna a primeira aresta que o vértice v participa ou
		// null se a lista de adjacência de v for vazia
	    Celula *item = this->adj[v]._primeiro();
	    return item != NULL ? new Aresta(v,item->vertice,item->peso) : NULL;
    }

	Aresta *proxAdj (int v) {
	    // Retorna a próxima aresta que o vértice v participa ou
	    // NULL se a lista de adjaência de v estiver no fim
	    Celula *item = this->adj[v].proximo();
	    return item != NULL ? new Aresta(v,item->vertice,item->peso) : NULL;
	}

	Aresta *retiraAresta (int v1, int v2) {
	    Celula chave(v2, 0);
	    Celula *item = this->adj[v1].retira(chave);
	    Aresta *aresta = item != NULL ? new Aresta(v1,v2,item->peso) : NULL;
	    delete item;
		return aresta;
	}

  	void imprime(){
	    for (int i = 0; i < this->numVertices; i++) {
		    cout << "Vertice " << i << ":" << endl;
		    Celula *item = this->adj[i]._primeiro ();
		    while (item != NULL) {
		      	cout << "  " << item->vertice << " (" <<item->peso<< ")" << endl;
		    	item = this->adj[i].proximo ();
		    }
	    }
  	}

	int _numVertices () {
		return this->numVertices;
	}

    ~Grafo() {
        delete[]this->adj;
    }

};

class Dijkstra {
	private:
	  int *antecessor; //vetor de antecessores dos vertices
	  int *p;//vetor de pesos do vertice

	public:
	Grafo *grafo;
	Dijkstra (Grafo *grafo) {
  		this->grafo = grafo;
		this->antecessor = NULL;
		this->p = NULL;
  	}


	void calculaDijkstra (int raiz) throw (logic_error) {

		int n = this->grafo->_numVertices();

	    if (this->p)
			delete [] this->p;
	    // vetor de peso dos vértices - no final do algoritmo ele estará marcado na
	    //posicao [0..n] o menor caminho do vertice inicial até cada vertice u em [0..n]
	    this->p = new int[n]; //
	    int *vs = new int[n]; // v\'ertices

	    if (this->antecessor)
			delete [] this->antecessor;
	    this->antecessor = new int[n];

	    for (int u = 0; u < n; u ++) {
	      this->antecessor[u] = -1;
	      p[u] = INFINITO;
	      vs[u] = u;
	    }

	    p[raiz] = 0;
	    FPNaoOrdenado *fila = new FPNaoOrdenado();

		int valorVertice;
	    int valorPeso;
	    for (int i = 0; i<n; i++){
	    	valorVertice = (int) vs[i];
	    	valorPeso = (int)p[i];
	    	fila->insere(valorVertice,valorPeso);
		}
	    while (!fila->vazio()){

	      	int u = fila->remove()._vertice(); //u é o numero do vertice extraido

	      	if (!this->grafo->listaAdjVazia (u)) {
	        	Grafo::Aresta *adj = grafo->primeiroListaAdj (u);
				while (adj != NULL) {
	          		int v = adj->_v2 ();
	          		//o vertice v, nao está mais na mesma posicao da fila, assim, o p[v] está incorreto
	          		//tenho q buscar a posicao de v na heap
	          		if (this->p[v] > (this->p[u] + adj->_peso ())) {
	            		antecessor[v] = u;
	            		fila->diminuiChave(v, this->p[u] + adj->_peso ());
	            		this->p[v] = this->p[u] + adj->_peso ();
	          		}else{

					}
	          		delete adj;
			  		adj = grafo->proxAdj (u);
	        	}
	      	}
	    }
	    delete [] vs;
	}

  	int _antecessor (int u) {
  		return this->antecessor[u];
	}
  	int _peso (int u){
	  	return this->p[u];
	}

  	~Dijkstra () {
  		this->grafo = NULL;
    	if (this->p)
			delete [] this->p;
  		if (this->antecessor)
		  	delete [] this->antecessor;
  }

};

// Função principal que monta o grafo, solicita o nó de origem e faz os cálculos do Dijkstra
void FuncaoPrincipal(char caminho[30]){
FILE *arquivoEntrada;

    char prefixo[10];
    int valor1, valor2, valor3;
    int no_origem = 0;
	int V = 0;
	int nArestas = 0;
	int raiz = 0;

    // Variáveis para medir o tempo de execução
    float tempo;
    clock_t t_inicio, t_fim;

	Grafo *grafo = new Grafo(V);

	//printf("\nDigite o nome do arquivo: ");
    //scanf("%s", &caminho);

	arquivoEntrada = abreArquivo('l', caminho);

	// Prepara o nome do arquivo de saída
	char *nomeArquivoSaida;
    // Renomeia arquivo de saída para não sobrescrever a entrada
    nomeArquivoSaida = strncat(caminho, ".out", 4);

    fscanf(arquivoEntrada, "%s", &prefixo);

   	// Construção do grafo para os casos não direcionados (ALUE e DMXA)
    if(strcmp(prefixo, "A") == 0) {
        while(!feof(arquivoEntrada)){
            fscanf(arquivoEntrada, "%s %d %d %d" , &prefixo, &valor1, &valor2, &valor3);

            if(strcmp(prefixo, "V") == 0){

                grafo = new Grafo(valor1);
                V = valor1;

            }
            if(strcmp(prefixo, "E") == 0){
               // printf("%s %d %d %d\n", prefixo, valor1, valor2, valor3);
                Grafo::Aresta *a = new Grafo::Aresta (valor1, valor2, valor3);
                grafo->insereAresta (a->_v1 (), a->_v2 (), a->_peso ());
                delete a;

            }
        }
        no_origem = NULL;
        printf("\nDigite o nó de origem entre 1 e %d: ", V);
        scanf("%d", &no_origem);


        // Executa o Dijkstra para os grafos esparsos (inicia no vértice 1)
        if (no_origem > 0 && no_origem <= V){
            t_inicio = clock(); // Guarda o horário do início da execução
            Dijkstra dj (grafo);

            int nV = grafo->_numVertices();
            //printf("nV %d \n", nV);
            dj.calculaDijkstra(no_origem);

            t_fim = clock(); // Guarda o horario do fim da execução

            tempo = (float)(t_fim - t_inicio)/CLOCKS_PER_SEC; // Calcula o tempo de execução

            // Gera o arquivo de saída
            FILE *arquivoSaida;

            arquivoSaida = abreArquivo('a', nomeArquivoSaida);

            // Imprime o tempo de execução
            fprintf(arquivoSaida, "\nTempo total de execução: %f segundo(s).\n\n", tempo);

            for (int i = 0; i < V; ++i){
                fprintf(arquivoSaida, "Origem: %i \t Destino: %d \t Distância: %d\n", no_origem, i, dj._peso(i));
            }

            fechaArquivo(arquivoSaida);
            printf("Cálculo completo. Arquivo de saída gerado");

        } else {
            printf("\nNó de origem inválido.");
        }
    } else if (strcmp(prefixo, "G") == 0){ // Construção do grafo para os casos direcionados (test-set1 e 2)
        while(!feof(arquivoEntrada)) {
            fscanf(arquivoEntrada, "%s %d %d %d" , &prefixo, &valor1, &valor2, &valor3);

            if(strcmp(prefixo, "V") == 0){
                printf("Total de vértices do grafo: %d \n\n", valor1);

                grafo = new Grafo(valor1);
                V = valor1;
            }
            if(strcmp(prefixo, "E") == 0){
                Grafo::Aresta *a = new Grafo::Aresta (valor1, valor2, valor3);
                grafo->insereAresta (a->_v1(), a->_v2(), a->_peso());

                delete a;
            }
        }
        printf("\nDigite o nó de origem entre 0 e %d: ", V);
        scanf("%d", &no_origem);

        // Executa o Dijkstra para os grafos completos (inicia no vértice 0)
        if (no_origem >= 0 && no_origem < V){
            t_inicio = clock(); // Guarda o horário do início da execução
            Dijkstra dj(grafo);

            int nV = grafo->_numVertices();
            //printf("nV %d \n", nV);
            dj.calculaDijkstra(0);

            t_fim = clock(); // Guarda o horario do fim da execução

            tempo = (float)(t_fim - t_inicio)/CLOCKS_PER_SEC; // Calcula o tempo de execução

            // Gera o arquivo de saída
            FILE *arquivoSaida;

            arquivoSaida = abreArquivo('a', nomeArquivoSaida);

            // Imprime o tempo de execução
            fprintf(arquivoSaida, "\nTempo total de execução: %f segundo(s).\n\n", tempo);

            for (int i = 0; i < V; ++i){
                fprintf(arquivoSaida, "Origem: %i \t Destino: %d \t Distância: %d\n", no_origem, i, dj._peso(i));
            }

            fechaArquivo(arquivoSaida);
            printf("Cálculo completo. Arquivo de saída gerado");

        } else {
            printf("\nNó de origem inválido.");
        }
    }

	fechaArquivo(arquivoEntrada);
    delete grafo;
}

// Função Menu
void Menu(){
    int opcao;
    char nomearquivo[30];
    //Menu
    do {

        printf("\n\n\t\tPrograma DIJKSTRA LISTA NÃO ORDENADA\n");
        printf("\nEscolha uma das opções abaixo:\n");
        printf("\n 1 - Calcular Dijkstra para um arquivo");
        printf("\n 2 - Sair\n\n");

        scanf("%d", &opcao);
        system("clear");


        switch(opcao){
            case 1:{
                    printf("\nDigite o nome do arquivo com extensão: ");
                    scanf("%s", &nomearquivo);
                    FuncaoPrincipal(nomearquivo);
                    continue;
                }
            case 2:
                exit(0);
            default:
                printf("Opção inválida! Escolha outra opção.\n\n");
        }
    }while(opcao != 2);
}

// Programa main
int main(){

	Menu();
    return 0;
}
