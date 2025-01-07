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


    # PARTE 2
    # TODO 1 - Centralidade de Proximidade: Implementar o cálculo da centralidade de proximidade para um vértice, 
    # usando fórmulas baseadas na soma das distâncias mínimas para outros vértices.

    # TODO 2 - Garantir que a funcionalidade lida com grafos desconexos e diferentes tipos de pesos.
    def centralidade_proximidade(self, vertice):
        # Calcula as distâncias e verifica se há ciclo negativo
        if not self.bellman_ford(vertice):
            return 0.0  # Retorna 0 se houver um ciclo negativo
        
        # Verificar as distâncias calculadas após Bellman-Ford
        print(f"Distâncias após Bellman-Ford para o vértice {vertice}: {self.distancias}")
        
        # Filtra apenas as distâncias finitas (alcançáveis) e diferentes de 0 (não considerar a distância para o próprio vértice)
        distancias_validas = [d for d in self.distancias if d != float('inf') and d != 0]

        # Número de vértices alcançáveis
        num_alcancaveis = len(distancias_validas)

        # Se não houver vértices alcançáveis, retorna 0
        if num_alcancaveis == 0:
            return 0.0

        soma_distancias = sum(distancias_validas)

        # Retorna a centralidade de proximidade
        return num_alcancaveis / soma_distancias






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
            print("11. Executar Algoritmo de Prim")
            print("12. Centralidade de Proximidade")
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
            elif opcao == "11":
                inicio = int(input("Digite o vértice inicial para o Algoritmo de Prim: "))
                resultado = self.prim(inicio)
                if resultado:
                    mst, custo_total = resultado
                    print("Árvore Geradora Mínima (MST):")
                    for origem, destino, peso in mst:
                        print(f"Aresta {origem} -> {destino} com peso {peso}")
                    print(f"Custo total da MST: {custo_total}")
                    # Salvando o resultado em arquivo no mesmo formato de entrada
                    nome_arquivo = "mst_resultado.txt"
                    with open(nome_arquivo, "w") as arquivo:
                        # Escreve o número de vértices
                        arquivo.write(f"{self.num_vertices}\n")

                        # Escreve as arestas da árvore geradora mínima (MST) no novo formato
                        for origem, destino, peso in mst:
                            arquivo.write(f"{origem} {destino} {peso:.1f}\n")

                        # Escreve o peso total da MST no arquivo
                        arquivo.write(f"Peso total: {custo_total:.1f}")

                    print(f"A árvore geradora mínima foi salva em '{nome_arquivo}'.")
            
            elif opcao == "12":
                vertice = int(input("Digite o vértice para calcular a centralidade de proximidade: "))
                centralidade = self.centralidade_proximidade(vertice)
                print(f"Centralidade de proximidade do vértice {vertice}: {centralidade:.4f}")

            elif opcao == "0":
                print("Saindo do programa...")
                break
            else:
                print("Opção inválida! Tente novamente.")

    def prim(self, inicio=1):
        import heapq

        inicio -= 1  # Ajusta para índice baseado em 0
        visitados = [False] * self.num_vertices
        heap = []  # Min-heap para selecionar arestas de menor peso
        mst = []   # Lista para armazenar as arestas da MST
        custo_total = 0  # Custo total da MST

        # Adiciona todas as arestas do vértice inicial ao heap
        visitados[inicio] = True
        for destino in range(self.num_vertices):
            peso = self.matriz_adj[inicio][destino]
            if peso != 0:
                heapq.heappush(heap, (peso, inicio, destino))

        # Enquanto o heap não estiver vazio e a MST não estiver completa
        while heap and len(mst) < self.num_vertices - 1:
            peso, origem, destino = heapq.heappop(heap)
            if not visitados[destino]:
                # Adiciona a aresta à MST
                mst.append((origem + 1, destino + 1, peso))  # Ajusta para índice 1
                custo_total += peso
                visitados[destino] = True

                # Adiciona as novas arestas do vértice destino ao heap
                for prox in range(self.num_vertices):
                    prox_peso = self.matriz_adj[destino][prox]
                    if prox_peso != 0 and not visitados[prox]:
                        heapq.heappush(heap, (prox_peso, destino, prox))

        # Verifica se todos os vértices foram visitados
        if len(mst) != self.num_vertices - 1:
            print("O grafo não é conectado, não foi possível formar uma MST.")
            return None

        return mst, custo_total

    def emparelhamento_maximo_edmonds(self):
        pares = [-1] * self.num_vertices  # Array para armazenar pares de emparelhamento
        base = list(range(self.num_vertices))  # Base de cada vértice
        pai = [-1] * self.num_vertices  # Pai no caminho aumentante
        visitado = [False] * self.num_vertices  # Visitado no BFS

        def lca(v, u):
            """Encontra o menor ancestral comum (Lowest Common Ancestor)."""
            marcados = [False] * self.num_vertices
            while True:
                v = base[v]
                marcados[v] = True
                if pares[v] == -1:
                    break
                v = pai[pares[v]]

            while True:
                u = base[u]
                if marcados[u]:
                    return u
                u = pai[pares[u]]

        def contrair(v, u, anc):
            """Contrai o blossom no grafo."""
            while base[v] != anc:
                pai[v] = u
                u = pares[v]
                if base[u] != anc:
                    base[u] = anc
                v = pai[u]

        def bfs(origem):
            """Busca em largura para encontrar caminhos aumentantes."""
            for i in range(self.num_vertices):
                pai[i] = -1
                base[i] = i
                visitado[i] = False

            fila = [origem]
            visitado[origem] = True

            while fila:
                v = fila.pop(0)
                for u in self.adj[v]:
                    if base[v] == base[u] or (pares[v] != -1 and pares[v] == u):
                        continue

                    if u == origem or (pares[u] != -1 and pai[pares[u]] != -1):
                        anc = lca(v, u)
                        contrair(v, u, anc)
                        contrair(u, v, anc)
                    elif pai[u] == -1:
                        pai[u] = v
                        if pares[u] == -1:
                            while u != -1:
                                v = pai[u]
                                prox = pares[v]
                                pares[v] = u
                                pares[u] = v
                                u = prox
                            return True
                        u = pares[u]
                        visitado[u] = True
                        fila.append(u)

            return False

        emparelhamento_maximo = 0
        for v in range(self.num_vertices):
            if pares[v] == -1 and bfs(v):
                emparelhamento_maximo += 1

        return emparelhamento_maximo, [(u, pares[u]) for u in range(self.num_vertices) if pares[u] > u]
