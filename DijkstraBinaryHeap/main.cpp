#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include <time.h>

// Struct para representar um nó na lista de adjacência
struct VerticeListaAdj
{
    int destino;
    int peso;
    struct VerticeListaAdj* prox;
};

// Struct para representar a lista de adjacência
struct ListaAdj
{
    struct VerticeListaAdj *inicio;  // ponteiro para o início da fila
};

// Struct para representar um grafo. Um grafo é um array de listas de adjacência.
// O tamanho do array é V (conjunto de vértices do grafo)
struct Grafo
{
    int V;
    struct ListaAdj* array;
};

// Função que cria um novo nó na lista de adjacência do grafo direcionado
struct VerticeListaAdj* insereVerticeListaAdj(int destino, int peso)
{
    struct VerticeListaAdj* insereVertice = (struct VerticeListaAdj*) malloc(sizeof(struct VerticeListaAdj));
    insereVertice->destino = destino;
    insereVertice->peso = peso;
    insereVertice->prox = NULL;
    return insereVertice;
}

// Função que cria um grafo com V vértices
struct Grafo* constroiGrafo(int V)
{
    struct Grafo* grafo = (struct Grafo*) malloc(sizeof(struct Grafo));
    grafo->V = V;

    // Cria um array de listas de adjacência.  O tamanho do array é igual a V
    grafo->array = (struct ListaAdj*) malloc(V * sizeof(struct ListaAdj));

     // Inicializa as listas de adjacência vazias e atribui o valor NULL ao nó inicial
    for (int i = 0; i < V; ++i)
        grafo->array[i].inicio = NULL;

    return grafo;
}


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
        printf("Nao foi possivel abrir o arquivo");
        exit(0);
	}
	return arquivo;
}

void fechaArquivo(FILE* arquivo)
{
    fclose(arquivo);
}



// Inclui uma aresta em no grafo (direcionado)
void insereAresta(struct Grafo* grafo, int origem, int destino, int peso)
{
    // Inclui uma aresta da origem ao destino. Um novo nó é adicionado à lista de adjacência
    // da origem.  O nó é colocado no início da lista
    struct VerticeListaAdj* insereVertice = insereVerticeListaAdj(destino, peso);
    insereVertice->prox = grafo->array[origem].inicio;
    grafo->array[origem].inicio = insereVertice;
}

// Struct que representa o nó min da heap
struct VerticeHeapMin
{
    int  v;
    int dist;
};

// Struct que representa um heap min
struct HeapMin
{
    int tamanho;      // Número de nós que a heap possui no momento
    int capacidade;  // Capacidade da heap min
    int *pos;     // Ponteiro necessário para a função diminuichave()
    struct VerticeHeapMin **array;
};

// Funçã que cria um novo nó no Heap min
struct VerticeHeapMin* insereVerticeHeapMin(int v, int dist)
{
    struct VerticeHeapMin* heapMinVertice =
           (struct VerticeHeapMin*) malloc(sizeof(struct VerticeHeapMin));
    heapMinVertice->v = v;
    heapMinVertice->dist = dist;
    return heapMinVertice;
}

// Função que cria a Heap min
struct HeapMin* constroiHeapMin(int capacidade)
{
    struct HeapMin* heapMin = (struct HeapMin*) malloc(sizeof(struct HeapMin));
    heapMin->pos = (int *)malloc(capacidade * sizeof(int));
    heapMin->tamanho = 0;
    heapMin->capacidade = capacidade;
    heapMin->array = (struct VerticeHeapMin**) malloc(capacidade * sizeof(struct VerticeHeapMin*));
    return heapMin;
}

// Função que troca dois nós no heap. Necessário para heap minify
void trocaVerticeHeapMin(struct VerticeHeapMin** a, struct VerticeHeapMin** b)
{
    struct VerticeHeapMin* t = *a;
    *a = *b;
    *b = t;
}

// Uma função para heapify um índice específico
// Esta função também atualiza a posição dos nós quando eles são trocados.
// A posição é necessária para a função diminuichave()
void heapfyMin(struct HeapMin* heapMin, int ind)
{
    int minimo, esq, dir;
    minimo = ind;
    esq = 2 * ind + 1;
    dir = 2 * ind + 2;

    if (esq < heapMin->tamanho &&
        heapMin->array[esq]->dist < heapMin->array[minimo]->dist )
      minimo = esq;

    if (dir < heapMin->tamanho &&
        heapMin->array[dir]->dist < heapMin->array[minimo]->dist )
      minimo = dir;

    if (minimo != ind)
    {
        // Os nós que devem ser trocados no heap min
        VerticeHeapMin *minimoVertice = heapMin->array[minimo];
        VerticeHeapMin *indVertice = heapMin->array[ind];

        // Troca as posições
        heapMin->pos[minimoVertice->v] = ind;
        heapMin->pos[indVertice->v] = minimo;

        // Troca de nós
        trocaVerticeHeapMin(&heapMin->array[minimo], &heapMin->array[ind]);

        heapfyMin(heapMin, minimo);
    }
}

// Função para checar se a heap min está vazia
int ehVazia(struct HeapMin* heapMin)
{
    return heapMin->tamanho == 0;
}

// Função para extrair o menor nó do heap
struct VerticeHeapMin* removeMin(struct HeapMin* heapMin)
{
    if (ehVazia(heapMin))
        return NULL;

    // Guarda o nó raiz
    struct VerticeHeapMin* raiz = heapMin->array[0];

    // Substitui o nó raiz pelo último nó
    struct VerticeHeapMin* ultimoVertice = heapMin->array[heapMin->tamanho - 1];
    heapMin->array[0] = ultimoVertice;

    // Atualiza a posição do último nó
    heapMin->pos[raiz->v] = heapMin->tamanho-1;
    heapMin->pos[ultimoVertice->v] = 0;

    // Reduz o tamanho do heap e a raiz do heapify
    --heapMin->tamanho;
    heapfyMin(heapMin, 0);

    return raiz;
}

// Função para decrementar o valor da distância de um vértice v. Esta função
// usa pos[] do heap min para pegar o índice atual do nó no heap min
void diminuiChave(struct HeapMin* heapMin, int v, int dist)
{
    // Pega o índice de v no heap array
    int i = heapMin->pos[v];

    // Pega o nó e atualiza o valor da distância
    heapMin->array[i]->dist = dist;

    // Caminha para cima enquanto a árvore completa não é inserida no heap.
    // Este é um laço de complexidade O(Logn)
    while (i && heapMin->array[i]->dist < heapMin->array[(i - 1) / 2]->dist)
    {
        // Troca o nó atual com o nó pai
        heapMin->pos[heapMin->array[i]->v] = (i-1)/2;
        heapMin->pos[heapMin->array[(i-1)/2]->v] = i;
        trocaVerticeHeapMin(&heapMin->array[i],  &heapMin->array[(i - 1) / 2]);

        // muda para o índice do pai
        i = (i - 1) / 2;
    }
}

// Função para checar se uma aresta
// 'v' está no heap min ou não
bool existeNoHeapMin(struct HeapMin *heapMin, int v)
{
   if (heapMin->pos[v] < heapMin->tamanho)
     return true;
   return false;
}

// Função para imprimir a solução
void imprime(int dist[], int n)
{
    printf("Menor distância do nó a partir da origem\n");
    for (int i = 0; i < n; ++i)
        printf("%d \t\t %d\n", i, dist[i]);
}

// A função principal que calcula as distâncias dos caminhos mais curtos da origem para todos
// os outros nós. É uma função com complexidade O(ELogV)
void dijkstra(struct Grafo* grafo, int origem)
{
    // Variáveis para medir o tempo de execução
    float tempo;
    clock_t t_inicio, t_fim;

    t_inicio = clock(); // Guarda o horário do início da execução


    int V = grafo->V;// Recebe o número de vértices do grafo
    int dist[V];      // Valores das distâncias usadas para escolher a aresta de menor peso

    // heapMin representa o conjunto E (arestas)
    struct HeapMin* heapMin = constroiHeapMin(V);

    // Inicializa o heap min com todos os vértices. atribui o valor da distância de todos os vértices
    for (int v = 0; v < V; ++v)
    {
        dist[v] = INT_MAX;
        heapMin->array[v] = insereVerticeHeapMin(v, dist[v]);
        heapMin->pos[v] = v;
    }

    // Faz com que o valor da distância do vértice de origem seja igual a 0 para que seja extraído primeiro
    heapMin->array[origem] = insereVerticeHeapMin(origem, dist[origem]);
    heapMin->pos[origem]   = origem;
    dist[origem] = 0;
    diminuiChave(heapMin, origem, dist[origem]);

    // Inicializa o tamanho do heap min igual a V
    heapMin->tamanho = V;

    // Vertice laço seguinte, heap min contém todos os nós
    // para os quais o  caminho mais curto ainda não foi finalizado.
    while (!ehVazia(heapMin))
    {
        // Extrai o vértice com o menor valor de distância
        struct VerticeHeapMin* heapMinVertice = removeMin(heapMin);
        int u = heapMinVertice->v; // Guarda o número do vértice extraído

        // Passa por todos os vértices visitados de u (o vértice extraído)
        // e atualiza os valores de suas distâncias
        struct VerticeListaAdj* visitado = grafo->array[u].inicio;
        while (visitado != NULL)
        {
            int v = visitado->destino;

            // Se a menor distância para v não está finalizada, e a distância para v
            // passando por u é menor que sua distância calculada anterior..
            if (existeNoHeapMin(heapMin, v) && dist[u] != INT_MAX && visitado->peso + dist[u] < dist[v])
            {
                dist[v] = dist[u] + visitado->peso;

                // ..atualiza o valor da distância no heap min também
                diminuiChave(heapMin, v, dist[v]);
            }
            visitado = visitado->prox;
        }
    }

    t_fim = clock(); // Guarda o horario do fim da execução

    tempo = (t_fim, t_inicio)*1000/CLOCKS_PER_SEC; // Calcula o tempo de execução


    FILE *arquivoSaida;
    arquivoSaida = abreArquivo('a',"/home/solange/Documentos/Trabalho Pratico/TrabalhoAPA/check/saida_check_v5_s2.txt");

    // Imprime o tempo de execução
    fprintf(arquivoSaida, "\nTempo total de execução: %f milissegundo(s).\n\n", tempo);

    // Imprime as menores distâncias calculadas
    for (int i = 0; i < V; ++i)
        fprintf(arquivoSaida, "%d \t %d\n", i, dist[i]);

    fechaArquivo(arquivoSaida);
}


// Programa main para testar as funções acima
int main()
{

    // testes com arquivos

	FILE *arquivoEntrada;
//	FILE *arquivoSaida;
    char prefixo[10];
    int valor1, valor2, valor3;
	int V;
    struct Grafo* grafo = constroiGrafo(0);

	arquivoEntrada = abreArquivo('l', "/home/solange/Documentos/Trabalho Pratico/TrabalhoAPA/check/check_v5_s2.dat");


	while(!feof(arquivoEntrada))
	{

        fscanf(arquivoEntrada, "%s %d %d %d" , &prefixo, &valor1, &valor2, &valor3);
        if(strcmp(prefixo, "V") == 0)
        {
            V = valor1;

            printf("Total de vértices do grafo: %d \n\n", V);
            grafo = constroiGrafo(V);
        }
        if(strcmp(prefixo, "E") == 0){
            insereAresta(grafo, valor1, valor2, valor3);
            //fprintf(arquivoSaida, "%s %d %d %d\n", prefixo, valor1, valor2, valor3);
        }
	}



	fechaArquivo(arquivoEntrada);
//	fechaArquivo(arquivoSaida);


    dijkstra(grafo, 0); // Executa o Dijkstra para o grafo

    return 0;
}
