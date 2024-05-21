import tkinter as tk
import customtkinter as ctk

ctk.set_appearance_mode("system")
ctk.set_default_color_theme("blue")

def slider_event(value):
    print(value)


app = ctk.CTk()
app.geometry("720x480")
app.title("Trying out new GUI")

# Label

title = ctk.CTkLabel(app, text='Insert some')
title.pack(padx=10, pady=10)

#Entry
link = ctk.CTkEntry(app, placeholder_text="CTkEntry")
link.pack()


#Button
download = ctk.CTkButton(app, text="Download")
download.pack(padx=10, pady=10)

#Slider
slider = ctk.CTkSlider(app, from_=0, to=100, command=slider_event)
slider.pack()


#Switch
def switch_event():   
    if switch_var.get() == "on":
        print("on")
    else:
        print("off")
    print("switch toggled, current value:", switch_var.get())

switch_var = ctk.StringVar(value="on")
switch = ctk.CTkSwitch(app, text="CTkSwitch", command=switch_event,
                                 variable=switch_var, onvalue="on", offvalue="off")

switch.pack()

#OptionMenu
def optionmenu_callback(choice):
    print("optionmenu dropdown clicked:", choice)

optionmenu_var = ctk.StringVar(value="option 2")
optionmenu = ctk.CTkOptionMenu(app,values=["option 1", "option 2"],
                                         command=optionmenu_callback,
                                         variable=optionmenu_var)

optionmenu.pack()

#Combobox
def combobox_callback(choice):
    print("combobox dropdown clicked:", choice)

combobox_var = ctk.StringVar(value="option 2")
combobox = ctk.CTkComboBox(app, values=["option 1", "option 2"],
                                     command=combobox_callback, variable=combobox_var)
combobox_var.set("option 2")

combobox.pack(padx=10, pady=10)

#Checkbox
def checkbox_event():
    print("checkbox toggled, current value:", check_var.get())

check_var = ctk.StringVar(value="on")
checkbox = ctk.CTkCheckBox(app, text="CTkCheckBox", command=checkbox_event,
                                     variable=check_var, onvalue="on", offvalue="off")
checkbox.pack()

#Progress bar

progressbar = ctk.CTkProgressBar(app, orientation="horizontal")
progressbar.pack()

#Radio Button

def radiobutton_event():
    print("radiobutton toggled, current value:", radio_var.get())

radio_var = tk.IntVar(value=0)
radiobutton_1 = ctk.CTkRadioButton(app, text="CTkRadioButton 1",
                                             command=radiobutton_event, variable= radio_var, value=1)
radiobutton_2 = ctk.CTkRadioButton(app, text="CTkRadioButton 2",
                                             command=radiobutton_event, variable= radio_var, value=2)

radiobutton_1.pack()
radiobutton_2.pack()

#segmented button

def segmented_button_callback(value):
    print("segmented button clicked:", value)

segemented_button_var = ctk.StringVar(value="Value 1")
segemented_button = ctk.CTkSegmentedButton(app, values=["Value 1", "Value 2", "Value 3"],
                                                     command=segmented_button_callback,
                                                     variable=segemented_button_var)

segemented_button.pack(padx=10, pady=10)

#tab view

tabview = ctk.CTkTabview(master=app)
tabview.pack(padx=20, pady=20)

tabview.add("tab 1")  # add tab at the end
tabview.add("tab 2")  # add tab at the end
tabview.set("tab 2")  # set currently visible tab

button = ctk.CTkButton(master=tabview.tab("tab 1"))
button.pack(padx=20, pady=20)

tabview.pack()

#Textbox
class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.grid_rowconfigure(0, weight=1)  # configure grid system
        self.grid_columnconfigure(0, weight=1)

        self.textbox = ctk.CTkTextbox(master=self, width=400, corner_radius=0)
        self.textbox.grid(row=0, column=0, sticky="nsew")
        self.textbox.insert("0.0", "Some example text!\n" * 50)


app = App()
app.mainloop()




#Scroll Bar for textbox
app = ctk.CTk()
app.grid_rowconfigure(0, weight=1)
app.grid_columnconfigure(0, weight=1)

# create scrollable textbox
tk_textbox = ctk.CTkTextbox(app, activate_scrollbars=False)
tk_textbox.grid(row=0, column=0, sticky="nsew")

# create CTk scrollbar
ctk_textbox_scrollbar = ctk.CTkScrollbar(app, command=tk_textbox.yview)
ctk_textbox_scrollbar.grid(row=0, column=1, sticky="ns")

# connect textbox scroll event to CTk scrollbar
tk_textbox.configure(yscrollcommand=ctk_textbox_scrollbar.set)

app.mainloop()






#Run app
app.mainloop()







#Frame

# frame = ctk.CTkFrame(master=app, width=200, height=200)

class MyFrame(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)

        # add widgets onto the frame, for example:
        self.label = ctk.CTkLabel(self, text='Insert some')
        self.label.grid(row=0, column=0, padx=20)


class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.geometry("400x200")
        self.grid_rowconfigure(0, weight=1)  # configure grid system
        self.grid_columnconfigure(0, weight=1)

        self.my_frame = MyFrame(master=self)
        self.my_frame.grid(row=0, column=0, padx=20, pady=20, sticky="nsew")

        self.my_frame2 = MyFrame(master=self)
        self.my_frame2.grid(row=0, column=0, padx=0, pady=10, sticky="nsew")


app = App()
app.mainloop()


#Scrollable Frame

class MyFrame(ctk.CTkScrollableFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)

        # add widgets onto the frame...
        self.label = ctk.CTkLabel(self)
        self.label.grid(row=0, column=0, padx=20)


class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        self.my_frame = MyFrame(master=self, width=300, height=200, corner_radius=0, fg_color="transparent")
        self.my_frame.grid(row=0, column=0, sticky="nsew")


app = App()
app.mainloop()







