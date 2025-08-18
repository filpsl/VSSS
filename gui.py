import subprocess
import tkinter as tk
from tkinter import messagebox

# caminhos
DIR = "Corobeu/"

ROBOTS = {
    "ares": f"{DIR}CorobeuARES.py",
    "mediaPipe": f"{DIR}Corobeu + MediaPipe.py",
    "ideal": f"{DIR}Corobeu_ideal.py",
    "kratos": f"{DIR}CorobeuKRATOS_ball.py",
    "zeus": f"{DIR}CorobeuZEUS_ball.py",
}

# dicionário de processos iniciados pela GUI
processes = {}

def run_robot(name):
    if name in processes and processes[name].poll() is None:
        messagebox.showinfo("Info", f"{name} já está rodando (PID={processes[name].pid})")
        return
    try:
        proc = subprocess.Popen(["python3", ROBOTS[name]])
        processes[name] = proc
        print(f"rodando {name} PID={proc.pid}")
    except Exception as e:
        messagebox.showerror("Erro", f"Não foi possível iniciar {name}:\n{e}")

def kill_robot(name):
    proc = processes.get(name)
    if proc and proc.poll() is None:
        proc.terminate()  # encerra graciosamente
        print(f"Morto {name} PID={proc.pid}")
        processes[name] = None
    else:
        messagebox.showinfo("Info", f"{name} não está rodando.")

def kill_all_running_robots_cmd():
    killed = []
    for name, path in ROBOTS.items():
        result = subprocess.run(
            ["pgrep", "-f", path],
            capture_output=True,
            text=True
        )
        if result.stdout.strip():  # se encontrou PIDs
            pids = result.stdout.strip().split("\n")
            for pid in pids:
                subprocess.run(["kill", "-9", pid])
                killed.append(f"{name} (PID={pid})")
    if killed:
        messagebox.showinfo("Info", "Robôs encerrados:\n" + "\n".join(killed))
    else:
        messagebox.showinfo("Info", "Nenhum robô rodando encontrado.")

# GUI
root = tk.Tk()
root.title("Controle de Robôs")

for name in ROBOTS:
    frame = tk.Frame(root)
    frame.pack(fill="x", padx=5, pady=2)

    label = tk.Label(frame, text=name, width=12, anchor="w")
    label.pack(side="left")

    run_btn = tk.Button(frame, text="Rodar", command=lambda n=name: run_robot(n))
    run_btn.pack(side="left", padx=5)

    kill_btn = tk.Button(frame, text="Matar", command=lambda n=name: kill_robot(n))
    kill_btn.pack(side="left", padx=5)

kill_all_btn = tk.Button(root, text="Matar Todos", command=kill_all_running_robots_cmd, fg="red")
kill_all_btn.pack(pady=5)

# Sair com segurança
def on_closing():
    if messagebox.askokcancel("Sair", "Deseja encerrar todos os robôs e sair?"):
        kill_all_running_robots_cmd()
        root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()
