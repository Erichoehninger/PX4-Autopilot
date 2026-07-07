from pyulog import ULog
import matplotlib.pyplot as plt
import numpy as np
import tkinter as tk
from tkinter import filedialog

# ------------------------------------------
# Escolher arquivo
# ------------------------------------------

root = tk.Tk()
root.withdraw()

filename = filedialog.askopenfilename(
    title="Selecione um log PX4",
    filetypes=[("PX4 Log", "*.ulg")]
)

if not filename:
    quit()

ulog = ULog(filename)

datasets = [d for d in ulog.data_list if d.name == "ekf_score"]

if len(datasets) == 0:
    raise RuntimeError("Nenhum tópico ekf_score encontrado.")
# ------------------------------------------
# Calcula os scores de todas as instâncias
# ------------------------------------------

all_scores = []

colors = ["red", "green", "purple"]

for ds in datasets:

    d = ds.data

    score = np.zeros_like(d["pos_test"])

    for values, weight in [
        #(d["vel_test"], 1.0),   # igual ao PX4 atual
        (d["pos_test"], 1.0),
        (d["hgt_test"], 0.5),
        (d["hdg_test"], 0.5),
        (d["pos_var"], 0.2),
        (d["vel_var"], 0.2),
    ]:
        mask = np.isfinite(values)
        score[mask] += values[mask] * weight

    all_scores.append({
        "id": int(d["instance_id"][0]),
        "time": d["timestamp"] * 1e-6,
        "score": score
    })
# ------------------------------------------
# Plotar cada instância
# ------------------------------------------

for ds in datasets:

    data = ds.data

    t = data["timestamp"] * 1e-6

    leader_id = data["timestamp_utc"]

    vel_test = data["vel_test"]
    pos_test = data["pos_test"]
    hgt_test = data["hgt_test"]
    hdg_test = data["hdg_test"]
    pos_var = data["pos_var"]
    vel_var = data["vel_var"]

    # mesmo algoritmo do PX4
    score = np.zeros_like(vel_test)

    for values, weight in [
        #(vel_test, 1.0),
        (pos_test, 1.0),
        (hgt_test, 0.5),
        (hdg_test, 0.5),
        (pos_var, 0.2),
        (vel_var, 0.2),
    ]:
        mask = np.isfinite(values)
        score[mask] += values[mask] * weight

    px4_id = int(data["instance_id"][0])

    fig, axs = plt.subplots(
        2,
        1,
        figsize=(13, 4),
        sharex=True,
        constrained_layout=True
    )

    fig.suptitle(
        f"PX4 de ID: {px4_id}",
        fontsize=18,
        fontweight="bold"
    )
    fig.subplots_adjust(hspace=0.18)

    plots = [
        ("Leader ID", leader_id),
        ("Score", score),
    ]

    for i, (ax, (title, values)) in enumerate(zip(axs, plots)):

        if title == "Leader ID":
            ax.step(
                t,
                leader_id,
                where="post",
                linewidth=2
            )

            ax_score = ax.twinx()

            for color, other in zip(colors, all_scores):

                interp_score = np.interp(
                    t,
                    other["time"],
                    other["score"]
                )

                ax_score.plot(
                    t,
                    interp_score,
                    color=color,
                    linewidth=1.5,
                    alpha=0.8,
                    label=f'Score PX4 {other["id"]}'
                )

            ax_score.set_ylabel("Score")
            ax_score.legend(loc="upper right")

            ax.set_yticks(sorted(np.unique(values)))
            ax.set_ylabel("Leader")
            ax.set_xlabel("Tempo (s)")
        else:
            ax.plot(
                t,
                values,
                linewidth=1.3
            )
            ax.set_ylabel(title)

        ax.grid(True)

    axs[-1].set_xlabel("Tempo (s)")
# ---------------------------------------------------------
# Janela mostrando onde houve divergência entre os PX4
# ---------------------------------------------------------

leader_series = []

for ds in datasets:
    leader_series.append({
        "px4": int(ds.data["instance_id"][0]),
        "t": ds.data["timestamp"] * 1e-6,
        "leader": ds.data["timestamp_utc"].astype(int)
    })

# usa o primeiro PX4 como referência temporal
t_ref = leader_series[0]["t"]

leaders_interp = []

for s in leader_series:
    interp = np.interp(
        t_ref,
        s["t"],
        s["leader"]
    ).round().astype(int)

    leaders_interp.append(interp)

leaders_interp = np.array(leaders_interp)

# verdadeiro quando existe discordância
disagreement = np.any(
    leaders_interp != leaders_interp[0],
    axis=0
)

fig, ax = plt.subplots(figsize=(14,3))

for i, s in enumerate(leader_series):
    ax.step(
        t_ref,
        leaders_interp[i] + i*0.08,   # pequeno offset para enxergar sobreposição
        where="post",
        linewidth=2,
        label=f'PX4 {s["px4"]}'
    )

# pinta as regiões onde existe divergência
inside = False
start = None

for i, d in enumerate(disagreement):

    if d and not inside:
        inside = True
        start = t_ref[i]

    elif not d and inside:
        ax.axvspan(start, t_ref[i], color="red", alpha=0.25)
        inside = False

if inside:
    ax.axvspan(start, t_ref[-1], color="red", alpha=0.25)

ax.set_title("Momentos em que houve discordância na eleição do líder")
ax.set_xlabel("Tempo (s)")
ax.set_ylabel("Leader ID")
ax.grid(True)
ax.legend()
plt.show()