import sys
sys.path.append(r"C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation")

import plotly.graph_objects as go
from dev.preprocessing.networks.network_manipulation import FullNetwork

""" this script plots the network in an interactive mode (node_indices are shown by hovering)"""

def show_interactive_network(network_path):
    nw = FullNetwork(nw_p)
    
    edge_x = []
    edge_y = []
    edge_text = []
    for edge in nw.edge_id_to_edge.values():
        x0, y0 = edge.start_node.coordinates
        x1, y1 = edge.end_node.coordinates
        
        edge_x.append(x0)
        edge_x.append(x1)
        edge_x.append(None)
        edge_y.append(y0)
        edge_y.append(y1)
        edge_y.append(None)
        
    edge_trace = go.Scatter(
        x=edge_x, y=edge_y,
        line=dict(width=0.5, color='#888'),
        hoverinfo='text',
        mode='lines')
    
    node_x = []
    node_y = []
    node_text = []
    i_to_index = {}
    node_coords_to_index = {}
    for i, node in enumerate(nw.nodes):
        x, y = node.coordinates
        if node_coords_to_index.get( (x,y) ) is None:
            i_to_index[i] = node.index
            node_x.append(x)
            node_y.append(y)
            node_coords_to_index[ (x,y) ] = len(node_text)
            node_text.append(f"{node.index}")
        else:
            ind = node_coords_to_index.get( (x,y) )
            node_text[ind] = node_text[ind] + f"_{node.index}"
        
    node_trace = go.Scatter(
        x=node_x, y=node_y,
        mode='markers',
        hoverinfo='text',
        marker=dict(
            showscale=True,
            # colorscale options
            #'Greys' | 'YlGnBu' | 'Greens' | 'YlOrRd' | 'Bluered' | 'RdBu' |
            #'Reds' | 'Blues' | 'Picnic' | 'Rainbow' | 'Portland' | 'Jet' |
            #'Hot' | 'Blackbody' | 'Earth' | 'Electric' | 'Viridis' |
            colorscale='YlGnBu',
            reversescale=True,
            color=[],
            size=10,
            colorbar=dict(
                thickness=15,
                title='Node Connections',
                xanchor='left',
                titleside='right'
            ),
            line_width=2))
    node_trace.text = node_text
    
    
    fig = go.Figure(data=[edge_trace, node_trace],  #, node_trace
                layout=go.Layout(
                    title='<br>Network graph made with Python',
                    titlefont_size=16,
                    showlegend=False,
                    hovermode='closest',
                    margin=dict(b=20,l=5,r=5,t=40),
                    annotations=[ dict(
                        text="Python code: <a href='https://plotly.com/ipython-notebooks/network-graphs/'> https://plotly.com/ipython-notebooks/network-graphs/</a>",
                        showarrow=False,
                        xref="paper", yref="paper",
                        x=0.005, y=-0.002 ) ],
                    xaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
                    yaxis=dict(showgrid=False, zeroline=False, showticklabels=False))
                    )
    fig.show()

if __name__ == "__main__":
    nw_p = r"C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation\FleetPy\data\networks\grafing_simplified_v3"
    show_interactive_network(nw_p)