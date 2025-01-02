from GrafoApp import GrafoApp
from Grafo import Grafo
import Arquivo
import tkinter as tk
from tkinter import messagebox, filedialog
import sys

def main():
    print("1 - Saida no terminal")
    print("2 - Saida por interface")
    saida = int(input("Escolha a saida desejada: "))

    match(saida):
        case 1:
            arquivo = input("Digite o nome do arquivo junto com o .txt: ")
            num_vertices = Arquivo.carregar_num_vertices(arquivo)

            print(f"Número de vértices: {num_vertices}")

            grafo = Grafo(num_vertices)

            Arquivo.carregar_arestas(arquivo, grafo)

            grafo.menu()

        case 2:
            GrafoApp.inicializa_interface()
        case _:
            print("Opção inválida. Encerrando o programa.")
            sys.exit(1)

if __name__ == "__main__":
    main()