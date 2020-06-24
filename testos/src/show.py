#!/usr/bin/env python

import rospkg
import plotly.graph_objects as go
import numpy as np

class Graph_3D(object):
    def __init__(self):
        pass
        
    def o_traj_pose(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('testos')
        path = pkg_path + '/src/traj_pose'
        
        pos = []
        with open(path,'r') as file:
            pos = [np.array(float(x)) for line in file for x in line.rstrip().split(',')]
        file.close()
        
        return pos

    def show_plot(self, pos, name):
        fig = go.Figure()

        for i in range (0,len(pos),3):
            if i == 0:
                color = "darkgreen"
                size = 8
                text = "START"
            elif i == len(pos)-3:
                color = "red"
                size = 8
                text = "GOAL"
            else:
                color = "black"
                size = 5
                text = ""

            fig.add_trace(go.Scatter3d(x=np.array(pos[i]), y=np.array(pos[i+1]), z=np.array(pos[i+2]),
                                        mode='markers',
                                        marker=dict(size=size,color=color),
                                        text=text,
                                        ))

        fig.update_layout(
            title_font_size=40,
            title={
                'text': 'Trajectory: ' + name,
                        'y':0.95,
                        'x':0.5,
                        'xanchor': 'center',
                        'yanchor': 'top'
                        },
                    xaxis=dict(title_text="Episodes",title_font_size=24),
                    yaxis=dict(title_text="Cumulative rewards",title_font_size=24),
                    scene = dict(
                        xaxis = dict(showgrid=True,
                                    showticklabels=True,
                                    zeroline=False,
                                    nticks=40,
                                    ),
                                    
                        yaxis = dict(showgrid=True,
                                    showticklabels=True,
                                    zeroline=False,
                                    nticks=40,
                                    ),
                        zaxis = dict(showgrid=True,
                                    showticklabels=True,
                                    zeroline=False,
                                    range=[0,1],
                                    nticks=40,
                                    ),

                                    ),
                    template="plotly_white",
                    showlegend=False,
                    autosize=True,
        )
        fig.show()
        
    