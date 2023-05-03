import simpleimageio as sio
import os, shutil, json
import numpy as np

def InitHtml():
    html = "<!DOCTYPE html><html><head>" + sio.make_header()
    html +="<style>" +\
    "    table {" +\
    "        border-collapse: collapse;" +\
    "        width: 100%;" +\
    "    }" +\
    "    th, td {" +\
    "        padding: 8px;" +\
    "        text-align: left;" +\
    "        border-bottom: 1px solid #DDD;" +\
    "    }" +\
    "    tr:hover {background-color: #D6EEEE;}" +\
    "    body {" +\
    "        max-width: 800px;" +\
    "        margin: 0 auto;" +\
    "    }" +\
    "</style>"
    html += "</head><body>"
    return html


def get_render_samples(f):
    with open(os.path.join(f, "Render.json")) as fp:
        file = json.load(fp)
    return file["NumIterations"]

def get_render_time(f):
    with open(os.path.join(f, "Render.json")) as fp:
        file = json.load(fp)
    return file["RenderTime"]

# Define layout for error strings
td = lambda txt: "<td>{}</td>".format(txt)
tr = lambda txt: "<tr>{}</tr>".format(txt)
th = lambda txt: "<tr>{}</tr>".format(txt)
table = lambda txt: "<table>{}</table>".format(txt)

# Specify input    
setups = [
    ("PT","EqualTime", [128,64,32,16],["Reference","Ours", "Vevoda et al.", "Baseline"]), 
    ("PT","EqualSpp", [128,64,32,16],["Reference","Ours", "Vevoda et al.", "Baseline"]), 
]

for integrator, baseline, grids, methods in setups:
    print(f"Start processing {baseline} {integrator}!")
    # Clean previous results
    if os.path.exists(os.path.join("supplemental",integrator, baseline)):
        shutil.rmtree(os.path.join("supplemental",integrator,baseline))
    os.makedirs(os.path.join("supplemental",integrator,baseline))
    
    # Get all scene files in the result folder
    scenes = []
    prefix = "../bin/Release/net7.0/Results/{}/{}".format(integrator,baseline)
    for filename in os.listdir(os.path.join(prefix)):
        if os.path.isdir(os.path.join(prefix,filename)):
            scenes.append(filename)
    print(scenes)

    for s in scenes:
        html = InitHtml()
        refImgPath = os.path.join(prefix, "{}/Reference.exr".format(s))
        # error for CVs in all resolution
        errCvs = [] 
        errVevodas= []
        errBases = []
        # For sorting the best ratio of our method
        errRatioCvs = [] 
        statsCvs = []
        statsBases = []
        statsVevodas = []
        for g in grids:
            refImg = sio.read(refImgPath)
            paths = [os.path.join(prefix, f"{s}/{g}/{methods[1]}"),
                     os.path.join(prefix, f"{s}/{g}/Vevoda et al"),
                     os.path.join(prefix, f"{s}/{methods[3]}")
                     ]

            imgs = [sio.read(os.path.join(path,"Render.exr")) for path in paths]

            errors = [
                (sio.relative_mse_outlier_rejection(m,refImg))
                for m in imgs
            ]
            
            statistics = []
            if baseline == "EqualTime":
                statistics = [get_render_samples(path) for path in paths]
            if baseline == "EqualSpp":
                statistics = [get_render_time(path) for path in paths]
                            
            errCvs.append(errors[0])
            errVevodas.append(errors[1])
            errBases.append(errors[2])    
            errRatioCvs.append(errors[2] / errors[0])       
        
            statsCvs.append(statistics[0])
            statsVevodas.append(statistics[1])
            statsBases.append(statistics[2])  
            
        # sorted grids based on errCv
        sorted_indices = np.argsort(errRatioCvs)
        sorted_indices = np.flip(sorted_indices)     
        for sorted_index in sorted_indices:
            # Get sorted arrays and append them into html
            g = grids[sorted_index]
            errCv = errCvs[sorted_index]
            errBase = errBases[sorted_index]
            errVevoda = errVevodas[sorted_index]
            
            # Attach images to html
            html += "<br /> Grid resolution: {}".format(g)
            imgs = [
                refImg,
                sio.read(os.path.join(prefix, f"{s}/{g}/{methods[1]}/Render.exr")),
                sio.read(os.path.join(prefix,f"{s}/{g}/Vevoda et al/Render.exr")),
                sio.read(os.path.join(prefix,f"{s}/{methods[3]}/Render.exr"))
            ]

            html += sio.make_flip_book(list(zip(methods, imgs)))
            
            stats_str = ""
            if baseline == "EqualSpp":
                stats_str = "Render Time (ms)"
            if baseline == "EqualTime":
                stats_str = "Equal Time Spp"
            html += table(th(td("") + td("Ours") + td("Vevoda et al.") + td("Baseline")) +
                    tr( # Attach error strings to html
                    td("Rel MSE")+\
                    td("{:.2e}({:.2f}x)".format(errCv,errBase / errCv))+\
                    td("{:.2e} ({:.2f}x)".format(errVevoda,errBase / errVevoda)) +\
                    td("{:.2e} ({:.2f}x)").format(errBase,errBase/errBase) 
                    )+
                    tr( # Attch number of samples to html
                    td(stats_str)+\
                    td(f"{statsCvs[sorted_index]}")+\
                    td(f"{statsVevodas[sorted_index]}") +\
                    td(f"{statsBases[sorted_index]}")
                    )                
                ) 
            
        # output different grid imgs to html for each scene
        output = os.path.join("supplemental",integrator,baseline,'{}.html'.format(s))
        f = open(output, 'x')
        f.write(html)
        f.close()
        
print("Finish processing!")