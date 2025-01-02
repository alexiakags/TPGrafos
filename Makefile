# Nome do arquivo principal
MAIN_FILE = main.py

# Detecta o comando correto para Python
ifeq ($(OS),Windows_NT)
    PYTHON = python
    RM = del /Q __pycache__ 2>NUL || true
    INSTALL_TKINTER = @echo "Tkinter já vem pré-instalado no Python no Windows"
else
    PYTHON = python3
    RM = rm -rf __pycache__
    INSTALL_TKINTER = sudo apt-get install -y python3-tk
endif

# Regras principais
run: install-tkinter
	$(PYTHON) $(MAIN_FILE)

clean:
	$(RM)

install-tkinter:
	@echo "Verificando e instalando tkinter, se necessário..."
	$(INSTALL_TKINTER)

help:
	@echo "Comandos disponíveis:"
	@echo "  make run           - Executa o programa"
	@echo "  make clean         - Remove arquivos de cache"
	@echo "  make install-tkinter - Instala o tkinter no Linux (se necessário)"
