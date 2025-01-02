from Grafo import Grafo

@staticmethod # define um método dentro de uma classe que não precisa acessar 
# ou modificar os atributos da classe ou da instância
def carregar_num_vertices(arquivo):
    # número de vértices na primeira linha do arquivo.
    with open(arquivo, 'r') as f:
        num_vertices = int(f.readline().strip())
    return num_vertices

def carregar_arestas(arquivo, grafo):
    # processa as arestas a partir da segunda linha do arquivo.
    with open(arquivo, 'r') as f:
        # ignora a primeira linha já que o grafo já foi inicializado
        next(f)
        for linha in f:
            origem, destino, peso = linha.split()
            grafo.adicionar_aresta(int(origem), int(destino), float(peso))


