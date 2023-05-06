import tkinter as tk

def increase_value(slider):
    current_value = slider.get()
    if current_value < 1.0:
        slider.set(current_value + 0.01)

def decrease_value(slider):
    current_value = slider.get()
    if current_value > 0.0:
        slider.set(current_value - 0.01)

root = tk.Tk()
root.title("Slider Demo")

sliders = [
    ("FL2", "fl2"),
    ("FL1", "fl1"),
    ("BL1", "bl1"),
    ("BL2", "bl2"),
    ("FR2", "fr2"),
    ("FR1", "fr1"),
    ("BR1", "br1"),
    ("BR2", "br2")
]

for i, (label_text, var_name) in enumerate(sliders[:4]):
    label = tk.Label(root, text=label_text)
    label.grid(row=0, column=i*2, columnspan=2, padx=10)

    slider = tk.Scale(root, from_=0, to=1, resolution=0.01, orient=tk.HORIZONTAL)
    slider.grid(row=2, column=i*2, columnspan=2, padx=10)

    increase_button = tk.Button(root, text="+", command=lambda s=slider: increase_value(s))
    increase_button.grid(row=1, column=i*2+1, padx=5, sticky="e")

    decrease_button = tk.Button(root, text="-", command=lambda s=slider: decrease_value(s))
    decrease_button.grid(row=1, column=i*2+0, padx=5, sticky="w")

for i, (label_text, var_name) in enumerate(sliders[4:]):
    label = tk.Label(root, text=label_text)
    label.grid(row=3, column=(i)*2, columnspan=2, padx=10)

    slider = tk.Scale(root, from_=0, to=1, resolution=0.01, orient=tk.HORIZONTAL)
    slider.grid(row=5, column=(i)*2, columnspan=2, padx=10)

    increase_button = tk.Button(root, text="+", command=lambda s=slider: increase_value(s))
    increase_button.grid(row=4, column=(i)*2+1, padx=5, sticky="e")

    decrease_button = tk.Button(root, text="-", command=lambda s=slider: decrease_value(s))
    decrease_button.grid(row=4, column=(i)*2+0, padx=5, sticky="w")

root.mainloop()

