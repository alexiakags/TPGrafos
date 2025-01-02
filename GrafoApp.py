from Grafo import Grafo
import Arquivo
import tkinter as tk
from tkinter import messagebox, filedialog
import sys

class GrafoApp:
    def __init__(self, master, grafo):
        self.master = master
        self.grafo = grafo
        self.input_action = None  # Guarda a ação que será executada após a entrada do usuário

        master.title("Análise de Grafos")
        master.configure(bg="#f0f4f8")

        # Título com estilo
        self.title_frame = tk.Frame(master, bg="#37474f", pady=10)
        self.title_frame.grid(row=0, column=0, columnspan=2, sticky="nsew")
        self.label = tk.Label(
            self.title_frame,
            text="Ferramenta de Análise de Grafos",
            bg="#37474f",
            fg="white",
            font=("Arial", 16, "bold")
        )
        self.label.pack()

        # Frame de botões
        self.buttons_frame = tk.Frame(master, bg="#f0f4f8")
        self.buttons_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nw")

        self.buttons = [
            ("Ordem do grafo", self.ordem),
            ("Tamanho do grafo", self.tamanho),
            ("Densidade do grafo", self.densidade),
            ("Obter vizinhos", self.obter_vizinhos),
            ("Lista de graus dos vértices", self.grau_vertices),
            ("Pontos de articulação", self.pontos_articulacao),
            ("Busca em largura", self.busca_em_largura),
            ("Componentes conexas", self.componentes_conexas),
            ("Detectar ciclo", self.detectar_ciclo),
            ("Distância e Caminho Mínimo", self.caminho_minimo),
            ("Sair", self.sair)
        ]

        for i, (text, func) in enumerate(self.buttons):
            button = tk.Button(
                self.buttons_frame,
                text=text,
                command=func,
                width=25,
                bg="#0288d1",
                fg="white",
                font=("Arial", 10, "bold"),
                activebackground="#01579b",
                activeforeground="white"
            )
            button.grid(row=i, column=0, pady=5, padx=5, sticky="w")

        # Área de resultados com rolagem
        self.result_frame = tk.Frame(master, bg="#f0f4f8")
        self.result_frame.grid(row=1, column=1, padx=10, pady=10, sticky="ne")

        self.result_label = tk.Label(
            self.result_frame,
            text="Resultados:",
            bg="#f0f4f8",
            font=("Arial", 12, "bold")
        )
        self.result_label.pack(anchor="w")

        self.text_scrollbar = tk.Scrollbar(self.result_frame)
        self.result_text = tk.Text(
            self.result_frame,
            height=20,
            width=50,
            bg="#ffffff",
            fg="#000000",
            font=("Courier", 10),
            bd=2,
            relief="groove",
            wrap="word",
            yscrollcommand=self.text_scrollbar.set
        )
        self.text_scrollbar.config(command=self.result_text.yview)
        self.text_scrollbar.pack(side="right", fill="y")
        self.result_text.pack(side="left", fill="both", expand=True)

        # Campo de entrada de dados do usuário
        self.input_frame = tk.Frame(master, bg="#f0f4f8", pady=10)
        self.input_frame.grid(row=2, column=0, columnspan=2, sticky="nsew")

        self.instruction_label = tk.Label(
            self.input_frame,
            text="Instruções aparecerão aqui.",
            bg="#f0f4f8",
            font=("Arial", 12)
        )
        self.instruction_label.pack(anchor="w")

        self.user_input = tk.Entry(
            self.input_frame,
            width=40,
            font=("Arial", 12),
            relief="solid"
        )
        self.user_input.pack(side="left", padx=10)

        self.submit_button = tk.Button(
            self.input_frame,
            text="Confirmar",
            command=self.process_input,
            bg="#0288d1",
            fg="white",
            font=("Arial", 10, "bold"),
            activebackground="#01579b",
            activeforeground="white"
        )
        self.submit_button.pack(side="left")

    # Funções principais
    def ordem(self):
        self.exibir_resultado(f"Ordem do grafo: {self.grafo.ordem()}")

    def tamanho(self):
        self.exibir_resultado(f"Tamanho do grafo: {self.grafo.tamanho()}")

    def densidade(self):
        self.exibir_resultado(f"Densidade do grafo: {self.grafo.calcular_densidade()}")

    def obter_vizinhos(self):
        self.instruction_label.config(text="Digite o vértice para obter os vizinhos:")
        self.input_action = lambda vertice: self.exibir_resultado(
            f"Vizinhos do vértice {vertice}: {self.grafo.obter_vizinhos(int(vertice))}"
        )

    def grau_vertices(self):
        graus = [sum(1 for j in range(self.grafo.num_vertices) if self.grafo.matriz_adj[i][j] != 0)
                 for i in range(self.grafo.num_vertices)]
        self.exibir_resultado("Graus dos vértices:\n" + "\n".join(
            f"Vértice {i + 1}: Grau {grau}" for i, grau in enumerate(graus)))

    def pontos_articulacao(self):
        pontos = self.grafo.ponto_articulacao()
        self.exibir_resultado(f"Pontos de articulação: {pontos}")

    def busca_em_largura(self):
        self.instruction_label.config(text="Digite o vértice inicial para a busca em largura:")
        self.input_action = lambda vertice: self.exibir_resultado(
            f"Busca em largura a partir do vértice {vertice}: {self.grafo.busca_em_largura(int(vertice))}"
        )

    def componentes_conexas(self):
        componentes = self.grafo.roy_componentes_conexas()
        self.exibir_resultado(f"O grafo possui {componentes} componente(s) conexa(s).")

    def detectar_ciclo(self):
        tem_ciclo = self.grafo.detectar_ciclo()
        self.exibir_resultado(f"O grafo {'contém' if tem_ciclo else 'não contém'} ciclos.")

    def caminho_minimo(self):
        self.instruction_label.config(
            text="Digite o vértice de origem para calcular o caminho mínimo:"
        )
        self.input_action = self._calcular_caminho_minimo

    def _calcular_caminho_minimo(self, origem):
        try:
            origem = int(origem)  # Converte o valor digitado para inteiro
            if self.grafo.bellman_ford(origem):  # Supondo que `bellman_ford` retorna True/False
                resultado = self.grafo.imprimir_caminhos(origem)  # Chama o método de impressão
                self.exibir_resultado(f"Caminhos mínimos a partir do vértice {origem}:\n{resultado}")
            else:
                self.exibir_resultado("Erro: Ciclo de peso negativo detectado no grafo!")
        except Exception as e:
            self.exibir_resultado(f"Erro ao calcular caminhos mínimos: {str(e)}")


    def sair(self):
        self.master.destroy()
        sys.exit()

    # Funções auxiliares
    def exibir_resultado(self, texto):
        self.result_text.delete("1.0", tk.END)
        self.result_text.insert(tk.END, texto + "\n")
        self.instruction_label.config(text="Instruções aparecerão aqui.")
        self.user_input.delete(0, tk.END)

    def process_input(self):
        entrada = self.user_input.get()
        if entrada and self.input_action:
            try:
                self.input_action(entrada)
                self.input_action = None
            except Exception as e:
                self.exibir_resultado(f"Erro: {e}")

    @staticmethod
    def inicializa_interface():
        try:
            # Criação da janela principal
            root = tk.Tk()
            root.withdraw()  # Oculta a janela principal
            
            # Garante que a janela está no topo
            root.lift()  # Traz a janela para frente
            root.attributes("-topmost", True)  # Força a janela a estar no topo
            root.update()  # Atualiza o estado da janela

            # Diálogo para selecionar arquivo
            arquivo_entrada = filedialog.askopenfilename(
                title="Selecione o arquivo de entrada",
                filetypes=(("Arquivos de Texto", "*.txt"), ("Todos os Arquivos", "*.*"))
            )

            if not arquivo_entrada:
                messagebox.showerror("Erro", "Nenhum arquivo especificado. Encerrando o programa.")
                return
            
            # Carrega o grafo do arquivo
            num_vertices = Arquivo.carregar_num_vertices(arquivo_entrada)
            grafo = Grafo(num_vertices)
            Arquivo.carregar_arestas(arquivo_entrada, grafo)

            # Torna a janela principal visível
            root.deiconify()

            # Inicializa a aplicação
            app = GrafoApp(root, grafo)

            # Inicia o loop da interface
            root.mainloop()

        except FileNotFoundError:
            messagebox.showerror("Erro", "Arquivo não encontrado. Encerrando o programa.")
        except Exception as e:
            messagebox.showerror("Erro", f"Ocorreu um erro: {str(e)}")


