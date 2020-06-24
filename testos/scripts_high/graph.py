#!/usr/bin/env python3

import gym
import plotly.graph_objects as go
import numpy as np

class Graph(object):
    def __init__(self, outdir,episodes, alg):
        self.outdir = outdir
        self.max_episodes = episodes
        self.tit_alg = alg

    def scatter(self, env):
        data_y = gym.wrappers.Monitor.get_episode_rewards(env)
        data_x = np.arange(self.max_episodes)
        
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=data_x, y=data_y,
                    mode='lines+markers',
                    line=dict(color='darkred', width=3),
                    marker=dict(size=10,color="black")
                    ))
        fig.update_xaxes(tickfont=dict(size=30),
                        showgrid=True, 
                        gridwidth=0.5, 
                        gridcolor='black',
                        zeroline=True,
                        zerolinecolor='black',
                        zerolinewidth=0.5,
                        showline=True, 
                        linewidth=5,
                        linecolor='black',
                        mirror=True,
                        )
        fig.update_yaxes(tickfont=dict(size=30),
                        showgrid=True,
                        gridwidth=0.5,
                        gridcolor='black',
                        zeroline=True,
                        zerolinecolor='black',
                        zerolinewidth=0.5,
                        showline=True, 
                        linewidth=5, 
                        linecolor='black', 
                        mirror=True,
                        )
        fig.update_layout(
            title_font_size=50,
            title={
                'text': self.tit_alg,
                'y':0.98,
                'x':0.5,
                'xanchor': 'center',
                'yanchor': 'top'
                },
            xaxis=dict(title_text="Episodes",title_font_size=35),
            yaxis=dict(title_text="Cumulative rewards",title_font_size=35),
            template="plotly_white",
            showlegend=False,
            autosize=True,
        )
        fig.show()