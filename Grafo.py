class Grafo:

    def __init__(self, num_vertices):
        # inicializa o grafo
        self.num_vertices = num_vertices
        self.num_arestas = 0 
        self.matriz_adj = [[0 for _ in range(num_vertices)] for _ in range(num_vertices)]
        self.dt = [float('inf')] * num_vertices  # inicializa com infinito
        self.rot = [-1] * num_vertices           # inicializa com -1
        self.tem_ciclo = None                    
        self.ciclo_negativo = 0                  

    def adicionar_aresta(self, origem, destino, peso):
        if origem < 1 or origem > self.num_vertices or destino < 1 or destino > self.num_vertices:
            print("Erro: vértice fora do intervalo")
            return
        # adiciona a aresta à matriz de valores e atualiza o contador de arestas do grafo
        self.matriz_adj[origem - 1][destino - 1] = peso
        self.matriz_adj[destino - 1][origem - 1] = peso
        self.num_arestas += 1

    def imprimir_grafo(self):

        #  imprime o número do vértice de cada coluna.
        print("   ", end="")
        for j in range(1, self.num_vertices + 1):
            print(f"{j:>4}", end="")
        print()
        
        # imprime a matriz com o número do vértice de cada linha
        for i in range(self.num_vertices):
            print(f"{i + 1:<3}", end="")
            for j in range(self.num_vertices):
                print(f"{self.matriz_adj[i][j]:>4.1f}", end="")
            print()

    def ponto_articulacao(self):
        def dfs(v, visitado, disc, low, parent, pontos):
            visitado[v] = True
            children = 0
            disc[v] = low[v] = self.tempo
            self.tempo += 1

            for i in range(self.num_vertices):
                if self.matriz_adj[v][i] != 0:  # se existe uma aresta
                    if not visitado[i]:  # se o vértice i não foi visitado
                        parent[i] = v
                        children += 1
                        dfs(i, visitado, disc, low, parent, pontos)

                        low[v] = min(low[v], low[i])

                        if parent[v] == -1 and children > 1:
                            pontos.add(v)
                        if parent[v] != -1 and low[i] >= disc[v]:
                            pontos.add(v)
                    elif i != parent[v]:  # atualizar low value para ciclo
                        low[v] = min(low[v], disc[i])

        visitado = [False] * self.num_vertices
        disc = [float('inf')] * self.num_vertices
        low = [float('inf')] * self.num_vertices
        parent = [-1] * self.num_vertices
        pontos = set()

        self.tempo = 0
        for i in range(self.num_vertices):
            if not visitado[i]:
                dfs(i, visitado, disc, low, parent, pontos)

        return [p + 1 for p in pontos]  # retorna os pontos de articulação (1-indexados)

    def bellman_ford(self, origem):
        origem -= 1  # ajusta para índice baseado em 0
        self.distancias = [float('inf')] * self.num_vertices
        self.predecessores = [-1] * self.num_vertices
        self.distancias[origem] = 0

        # relaxa as arestas |V| - 1 vezes
        for _ in range(self.num_vertices - 1):
            for u in range(self.num_vertices):
                for v in range(self.num_vertices):
                    if self.matriz_adj[u][v] != 0:  # existe uma aresta
                        peso = self.matriz_adj[u][v]
                        if self.distancias[u] + peso < self.distancias[v]:
                            self.distancias[v] = self.distancias[u] + peso
                            self.predecessores[v] = u

        # verifica ciclos negativos
        for u in range(self.num_vertices):
            for v in range(self.num_vertices):
                if self.matriz_adj[u][v] != 0:
                    peso = self.matriz_adj[u][v]
                    if self.distancias[u] + peso < self.distancias[v]:
                        print("Ciclo de peso negativo detectado!")
                        return False

        return True

    def imprimir_caminhos(self, origem):
        origem -= 1  # Ajusta para índice baseado em 0
        resultado = f"Caminhos mínimos a partir do vértice {origem + 1}:\n"
        
        for v in range(self.num_vertices):
            if self.distancias[v] == float('inf'):
                resultado += f"Vértice {v + 1} não é alcançável\n"
            else:
                caminho = []
                atual = v
                while atual != -1:  # Reconstrói o caminho até a origem
                    caminho.append(atual + 1)  # Ajusta para índice 1
                    atual = self.predecessores[atual]
                caminho.reverse()  # Reverte para obter o caminho na ordem correta
                resultado += f"Para {v + 1}: distância = {self.distancias[v]:.1f}, caminho = {' -> '.join(map(str, caminho))}\n"

        return resultado

    
    # TODO 1 - implementar cálculo da densidade ε(G) com a fórmula ε(G) = 2 * |E| / (|V| * (|V| - 1)).
    def calcular_densidade(self):
        if self.num_vertices < 2:
            return 0
        return 2 * self.num_arestas / (self.num_vertices * (self.num_vertices - 1))
    
    # TODO 2 - criar função que retorna a lista de vizinhos de um vértice fornecido.
    def obter_vizinhos(self, vertice):
        vertice-=1
        vizinhos = []
        for i in range(self.num_vertices):
            if self.matriz_adj[vertice][i] != 0:
                vizinhos.append(i+1)   
        return vizinhos
    
    # TODO 3 - implementar função que detecta a presença de ciclos no grafo usando DFS (busca em profundidade).
    def detectar_ciclo(self):
        visitados = set()

        def dfs(v, pai):
            visitados.add(v)
            for vizinho in self.obter_vizinhos(v):
                if vizinho not in visitados:
                    if dfs(vizinho, v):
                        return True
                elif vizinho != pai:
                    return True
            return False

        for vertice in range(self.num_vertices):
            if vertice not in visitados:
                if dfs(vertice, -1):  # O vértice inicial não tem pai
                    return True
        return False



    def tamanho(self):
            # retorna o tamanho do grafo
            return self.num_arestas

    def ordem(self):
            # retorna a ordem do grafo
            return self.num_vertices

    def busca_em_largura(self, inicial):

        # realiza a busca em largura no grafo a partir do vértice inicial.
        # retorna:
        # sequência de vértices visitados
        # arestas da árvore de busca
        # arestas que não fazem parte da árvore de busca

        if inicial < 1 or inicial > self.num_vertices:
            print("Erro: vértice inicial fora do intervalo")
            return [], [], []

        visitados = [False] * self.num_vertices
        fila = []
        sequencia_visitada = []
        arestas_arvore = []
        arestas_nao_arvore = []

        # converte o índice do vértice para base 0
        inicial -= 1
        visitados[inicial] = True
        fila.append(inicial)

        processadas = set()

        while fila:
            vertice_atual = fila.pop(0)
            sequencia_visitada.append(vertice_atual + 1)  # voltar para base 1

            for i in range(self.num_vertices):
                if self.matriz_adj[vertice_atual][i] > 0:  # existe aresta
                    aresta = tuple(sorted((vertice_atual + 1, i + 1)))  # base 1, aresta ordenada
                    if aresta in processadas:
                        continue
                    processadas.add(aresta)

                    if not visitados[i]:
                        # aresta da árvore de busca
                        visitados[i] = True
                        fila.append(i)
                        arestas_arvore.append(aresta)
                    else:
                        # aresta que não pertence à árvore
                        arestas_nao_arvore.append(aresta)

        return sequencia_visitada, arestas_arvore, arestas_nao_arvore


    # TODO 1 - representação de vértices (calcular grau dos vértices)
    def grau_vertices(self):
        graus = []
        for i in range(self.num_vertices):
            grau = 0
            for j in range(self.num_vertices):
                if self.matriz_adj[i][j] != 0:
                    grau += 1
            graus.append(grau)
        #print()
        for i in range(self.num_vertices):
            print(f"Vértice {i+1} tem grau {graus[i]}")
        print()

    # TODO 2 - algoritmo de Roy para contar o número de componentes conexas (não é dfs)
    def roy_componentes_conexas(self):
        # cria matriz de alcançabilidade com base na matriz de adjacência
        alcançavel = [[1 if i == j or self.matriz_adj[i][j] != 0 else 0 for j in range(self.num_vertices)] for i in range(self.num_vertices)]
        
        # algoritmo de Floyd-Warshall para calcular a alcançabilidade
        for k in range(self.num_vertices):
            for i in range(self.num_vertices):
                for j in range(self.num_vertices):
                    alcançavel[i][j] = alcançavel[i][j] or (alcançavel[i][k] and alcançavel[k][j])

        # determina os componentes conexos
        visitados = [False] * self.num_vertices
        componentes = 0

        for i in range(self.num_vertices):
            if not visitados[i]:
                componentes += 1
                for j in range(self.num_vertices):
                    if alcançavel[i][j]:
                        visitados[j] = True

        # retorna o número de componentes conexas
        return componentes



    def menu(self):
        while True:
            print("\n" + "="*40)
            print("                   MENU")
            print("="*40)
            print("1. Retornar a ordem do grafo")
            print("2. Retornar o tamanho do grafo")
            print("3. Retornar a densidade do grafo (ε(G))")
            print("4. Retornar os vizinhos de um vértice")
            print("5. Determinar o grau de um vértice")
            print("6. Verificar se um vértice é articulação")
            print("7. Busca em largura e arestas fora da árvore")
            print("8. Determinar componentes conexas")
            print("9. Verificar se o grafo possui ciclo")
            print("10. Determinar distância e caminho mínimo")
            print("0. Sair")
            print("="*40)
            opcao = input("Escolha uma opção: ")

            if opcao == "1":
                print("Ordem do grafo:", self.ordem())
            elif opcao == "2":
                print("Tamanho do grafo:", self.tamanho())
            elif opcao == "3":
                print(f"Densidade do grafo:", self.calcular_densidade())
            elif opcao == "4":
                vizinhos = self.obter_vizinhos(int(input("Digite o vértice para obter os vizinhos: ")))
                print(f"Vizinhos do vértice: {vizinhos}")
            elif opcao == "5":
                print("Lista de graus dos vértices: ")
                self.grau_vertices()
            elif opcao == "6":
                ponto_articulacao = self.ponto_articulacao()
                print(f"Número de pontos de articulação: {len(ponto_articulacao)}")
                print(f"Pontos de articulação: {ponto_articulacao}")
            elif opcao == "7":
                vertice_inicial = int(input("Digite o vértice inicial para a busca em largura: "))
                sequencia, arvore, nao_arvore = self.busca_em_largura(vertice_inicial)
                print("Sequência de vértices visitados:", sequencia)
                print("Arestas da árvore de busca:", arvore)
                print("Arestas que não fazem parte da árvore de busca:", nao_arvore)
            elif opcao == "8":
                print(self.roy_componentes_conexas())
            elif opcao == "9":
                print(f"Ciclo presente no grafo: {'Sim' if self.detectar_ciclo() else 'Não'}")
            elif opcao == "10":
                 # Distância e Caminho Mínimo
                origem = 1  # Vértice inicial para o cálculo de caminhos mínimos
                print("Digite a origem: ")
                origem = int(input())
                print("\nCalculando distâncias e caminhos mínimos...")
                if(self.bellman_ford(origem)):
                    result = self.imprimir_caminhos(origem)
                    print(result)
            elif opcao == "0":
                print("Saindo do programa...")
                break
            else:
                print("Opção inválida! Tente novamente.")
