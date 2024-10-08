##MODULES


from math import *
import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy
import matplotlib.patches as patches
from random import *
import matplotlib as mpl
import matplotlib.animation as anim  #Fonction pour l'animation
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import axes3d  # Fonction pour la 3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import os


##APPELS DE FONCTIONS

#trajet (20,20,250,[[0,10],[20,5],[5,20],[20,15]])
#trajet (40,40,500,[[0,20]])

##GRADIENT

#problème des mes fonctions, pas de recherche de minimum si la dérivée vaut 0 au point de départ. Mais
#les fonctions de coût n'ont pas de maximum, et très peu probable de tomber sur un point de dérivée nulle au hasard

def gradient1D(f, a, pas, eps, max):
    grad= 1
    i=0
    while abs(grad)>eps:
        grad = (f(a+eps)-f(a-eps))/(2*eps) #calcul dérivée
        a = a-pas*grad
        i += 1
        if i > max:
            break
    return a

def gradient2D(f, x, y, pas, eps, max):
    grad=1
    i=0
    while grad>eps:
        gradx=(f(x+eps,y)-f(x-eps,y))/(2*eps) #calcul dérivée selon x
        grady=(f(x, y+eps)-f(x, y-eps))/(2*eps) #calcul dérivée selon y
        grad=sqrt(gradx**2+grady**2)
        x=x-pas*gradx
        y=y-pas*grady
        i+=1
        if i > max:
            break
    return x,y

def gradientND(f, nuplet, pas, eps, max):
    j=0
    grad=1
    gradi=0
    stock=deepcopy(nuplet) #stockage des variables modifiées à chaque étape
    while grad>eps:
        for i in range(len(nuplet)):
            t1=deepcopy(nuplet) #copie profonde des variables
            t1[i]+=eps
            t2=deepcopy(nuplet) #copie profonde des variables
            t2[i]-=eps
            gradi=(f(t1)-f(t2))/(2*eps)
            stock[i]-=pas*gradi
        nuplet=deepcopy(stock) #récupération des variables modifiées
        calcul=[nuplet[i]**2 for i in range(len(nuplet))] #passage au carré des valeurs pour calculer la norme
        grad=sqrt(sum(calcul))
        j+=1
        if j>max:
            break #en cas de dépassement de la limite d'itérations "max"
    return nuplet

def gradientNDmodif(f, nuplet, portes,obstacles,voisins,pas, eps, max):
    j=0
    normegradient=1
    coordonnees=deepcopy(nuplet) #stockage des variables modifiées à chaque étape
    tableau=[nuplet]
    while j<max:
        for i in range(len(nuplet)):
            tplus=deepcopy(nuplet) #copie profonde des variables
            tplus[i]+=eps
            tmoins=deepcopy(nuplet) #copie profonde des variables
            tmoins[i]-=eps
            deriveepartielle=(f(tplus,portes,obstacles,voisins)-f(tmoins,portes,obstacles,voisins))/(2*eps)
            if abs(pas*deriveepartielle)>eps:
                if pas*deriveepartielle<0:
                    deplacement=-eps
                else:
                    deplacement=eps
            else:
                deplacement=pas*deriveepartielle
            coordonnees[i]-=deplacement
        nuplet=deepcopy(coordonnees) #récupération des variables modifiées
        j+=1
        tableau.append(nuplet)
        if tableau[-1]==tableau[-2]:
            for i in range(len(nuplet)):
                nuplet[i]-=pas
    return tableau

##FONCTION DE COÛT


#trajectoire droite

def fcout(l,portes,obstacles,voisins):
    x=l[0]
    y=l[1]
    return min([sqrt((x-portes[i][0])**2+(y-portes[i][1])**2)
    +passageobstacleexp(obstacles,x,y)+coutvoisins(voisins,x,y) for i in range(len(portes))])

def passageobstacleconstant(obstacles,x,y):
    cout=0
    for elt in obstacles:
        if elt[0]=='cercle':                                        #obstacles ronds
            if (x-elt[1])**2+(y-elt[2])**2<elt[3]**2:
                cout+=30    #valeur arbitraire
        if elt[0]=='rectangle':                                     #obstacles rectangulaires
            if elt[1]<x<elt[1]+elt[3] and elt[2]<y<elt[2]+elt[4]:
                cout+=30    #valeur arbitraire
    return cout

def passageobstacleexp(obstacles,x,y):
    cout=0
    for elt in obstacles:
        if elt[0]=='cercle':
            cout+=30*exp(-elt[3]*(0.1*(x-elt[1])**2+1.8*(y-elt[2])**2))
    return cout

def coutvoisins(voisins,x,y):
    cout=0
    for elt in voisins:
        cout+=0.5*exp(-3*((x-elt[0])**2+(y-elt[1])**2))
    return cout


# def trajet_du_point(l,portes,obstacles):
#     x=l[0]
#     y=l[1]
#
#     for j in range(len(portes)):
#
#     traj=[] #tableau avec différent trajets
#     #................
#
#     return min(traj)
#fonctions prenant en compte les obstacles

#def choixtrajectoire(x,y,obs,)




##TRAJET

def trajet(longueur, largeur,n,portes,obstacles):
    X=[uniform(0,longueur) for i in range(n)]
    Y=[uniform(0,largeur) for i in range(n)]
    for i in range(len(X)):
        while passageobstacleconstant(obstacles,X[i],Y[i])!=0:
            X[i]=X[i]+1
    trajectoires=[]
    for i in range(len(X)):
        print(i)
        trajectoires.append(gradientNDmodif(fcout,[X[i],Y[i]],portes,obstacles,[],0.01,0.05,5000))
    print([trajectoires[i][-1] for i in range(len(X))])
    dist=[0.3 for i in range(len(X))]
    listeX=[X]
    listeY=[Y]
    abs=[0]
    ord=[0]
    absancien=X.copy()
    ordancien=Y.copy()
    marqueur=0
    while abs!=[]:
        print(marqueur)
        marqueur+=1
        abs=[]
        ord=[]
        for i in range(len(trajectoires)):
            j=0
            while j<len(trajectoires[i])-1 and ((trajectoires[i][j][0]-X[i])**2+(trajectoires[i][j][1]-Y[i])**2)<dist[i]**2:
                j+=1
            a,o=trajectoires[i][j]
            distporte=min([(a-portes[k][0])**2+(o-portes[k][1])**2 for k in range(len(portes))])
            if distporte>2:
                ref=True
                for k in range(len(absancien)):#y-a-t'il quelqu'un devant à l'instant précédent :
                    if i!=k and ((a-absancien[k])**2+(o-ordancien[k])**2)<2 and fcout([absancien[k],ordancien[k]],portes,obstacles,voisins)<fcout([a,o],portes,obstacles,voisins):
                        ref=False
                if ref:#si il n'y a personne, on avance
                    c=a
                    d=o
                    dist[i]+=0.3
                else:#sinon, on ne bouge pas
                    c=absancien[i]
                    d=ordancien[i]
                absancien[i]=c
                ordancien[i]=d
                abs.append(c)#on ajoute les coordonnées calculées si personne devant, on ne bouge pas sinon
                ord.append(d)
            else:
                absancien[i]=-100
                ordancien[i]=-100
        listeX.append(abs)
        listeY.append(ord)
    affichage2(longueur,largeur,listeX,listeY,portes,obstacles)#affichage de l'avancement des piétons

# def trajet2faux(longueur, largeur,n,portes,obstacles):
#     X=[uniform(0,longueur) for i in range(n)]
#     Y=[uniform(0,largeur) for i in range(n)]
#     for i in range(len(X)):
#         while passageobstacleconstant(obstacles,X[i],Y[i])!=0:
#             X[i]=X[i]+1
#     trajectoires=[]
#     for i in range(len(X)):
#         trajectoires.append(gradientNDmodif(fcout,[X[i],Y[i]],portes,obstacles,voisins,0.02,0.05,100))
#     dist=[0.3 for i in range(len(X))]
#     listeX=[X]
#     listeY=[Y]
#     abs=[0]
#     ord=[0]
#     absancien=X.copy()
#     ordancien=Y.copy()
#     marqueur=0
#     while abs!=[]:
#         print(marqueur)
#         print(len(dist))
#         print(len(trajectoires))
#         marqueur+=1
#         abs=[]
#         ord=[]
#         XX=X.copy()
#         YY=Y.copy()
#         distdist=dist.copy()
#         for i in range(len(trajectoires)):
#             j=0
#             while j<(len(trajectoires[i])-1) and ((trajectoires[i][j][0]-X[i])**2+(trajectoires[i][j][1]-Y[i])**2)<dist[i]**2:
#                 j+=1
#             a,o=trajectoires[i][j]
#             distporte=min([(a-portes[k][0])**2+(o-portes[k][1])**2 for k in range(len(portes))])
#             if distporte>2:
#                 ref=True
#                 for k in range(len(absancien)):#y-a-t'il quelqu'un devant à l'instant précédent :
#                     if i!=k and ((a-absancien[k])**2+(o-ordancien[k])**2)<2 and fcout([absancien[k],ordancien[k]],portes,obstacles,voisins)<fcout([a,o],portes,obstacles,voisins):
#                         ref=False
#                 if ref:#si il n'y a personne, on avance
#                     c=a
#                     d=o
#                     dist[i]+=0.3
#                 else:#sinon, on ne bouge pas
#                     c=absancien[i]
#                     d=ordancien[i]
#                 absancien[i]=c
#                 ordancien[i]=d
#                 abs.append(c)#on ajoute les coordonnées calculées si personne devant, on ne bouge pas sinon
#                 ord.append(d)
#             else:
#                 absancien[i]=-100
#                 ordancien[i]=-100
#                 distdist.pop(i)
#                 XX.pop(i)
#                 YY.pop(i)
#         listeX.append(abs)
#         listeY.append(ord)
#         X=XX.copy()
#         Y=YY.copy()
#         dist=distdist.copy()
#         trajectoires=[]
#         for i in range(len(abs)):
#             trajectoires.append(gradientNDmodif(fcout,[abs[i],ord[i]],portes,obstacles,voisins,0.01,0.05,100+20*marqueur))
#     print(listeX)
#     print(listeY)
#     affichage2(longueur,largeur,listeX,listeY,portes,obstacles)#affichage de l'avancement des piétons

def trajetaleatoire(longueur, largeur,n,portes,obstacles):
    X=[uniform(0,longueur) for i in range(n)]
    Y=[uniform(0,largeur) for i in range(n)]
    for i in range(len(X)):
        while passageobstacleconstant(obstacles,X[i],Y[i])!=0:
            X[i]=X[i]+1
    dist=[0.3 for i in range(len(X))]
    listeX=[X]
    listeY=[Y]
    abs=X.copy()
    ord=Y.copy()
    absancien=X.copy()
    ordancien=Y.copy()
    marqueur=0
    m=[True for i in range(len(X))]
    while abs!=[]:
        print(marqueur)
        marqueur+=1
        trajectoires=[]
        count=0
        for i in range(len(X)):
            voisins=[[absancien[j],ordancien[j]] for j in range(len(absancien))]
            voisins.pop(i)
            if m[i]:
                trajectoires.append(gradientNDmodif(fcout,[abs[i-count],ord[i-count]],portes,obstacles,voisins,0.01+0.00005*marqueur,0.05,100))
            else:
                trajectoires.append([])
                count+=1
        abs=[]
        ord=[]
        for i in range(len(trajectoires)):
            if m[i]:
                j=0
                while j<len(trajectoires[i])-1 and ((trajectoires[i][j][0]-X[i])**2+(trajectoires[i][j][1]-Y[i])**2)<dist[i]**2:
                    j+=1
                a,o=trajectoires[i][j]
                distporte=min([(a-portes[k][0])**2+(o-portes[k][1])**2 for k in range(len(portes))])
                if distporte>2:
                    ref=True
                    for k in range(len(absancien)):#y-a-t'il quelqu'un devant à l'instant précédent :
                        if i!=k and ((a-absancien[k])**2+(o-ordancien[k])**2)<2 and fcout([absancien[k],ordancien[k]],portes,obstacles,voisins)<fcout([a,o],portes,obstacles,voisins):
                            ref=False
                    if ref:#si il n'y a personne, on avance
                        c=a+uniform(-0.01,0.01)
                        d=o+uniform(-0.01,0.01)
                        dist[i]+=0.3
                    else:#sinon, on ne bouge pas
                        c=absancien[i]
                        d=ordancien[i]
                    absancien[i]=c
                    ordancien[i]=d
                    abs.append(c)#on ajoute les coordonnées calculées si personne devant, on ne bouge pas sinon
                    ord.append(d)
                else:
                    absancien[i]=-100
                    ordancien[i]=-100
                    m[i]=False
        listeX.append(abs)
        listeY.append(ord)
    affichage2(longueur,largeur,listeX,listeY,portes,obstacles)#affichage de l'avancement des piétons



##AFFICHAGE

def affichage(xmax,ymax,X,Y):
    plt.clf()
    fig, ax = plt.subplots()
    ax.add_patch(patches.Rectangle((-0.5,45),1,10,edgecolor = 'red',facecolor = 'red',fill=True))#rectangle d'arrivée
    ax.add_patch(patches.Rectangle((0,-10),xmax+10,ymax+20,edgecolor = 'red',fill=False))#murs de la pièce
    plt.axis([-5, xmax+15,-15, ymax+15])#limites des axes
    plt.scatter(X, Y, s=50, color=[0,0,0], marker=None, cmap=None, norm=None, vmin=None, vmax=None, alpha=1, linewidths=None, edgecolors=None, plotnonfinite=False, data=None)#points
    plt.show()

def affichage2(longueur, largeur, listeX,listeY,portes,obstacles):
    dimension=10/(longueur+largeur)
    listeXY=[]
#liste des listes des couples (x,y) de chacun de mes points :
    for i in range(len(listeX)):
        listeXY.append([[listeX[i][j],listeY[i][j]] for j in range(len(listeX[i]))])
    plt.close('all')
#dimension de la salle :
    fig=plt.figure(figsize=[8,5])
    axSalle=plt.subplot2grid((1,1),(0,0))
    axSalle.axis([-0.2*longueur, longueur*1.2,-0.2*largeur, largeur*1.2])
#affichage des portes :
    for i in range(len(portes)):
        axSalle.add_patch(patches.Rectangle((portes[i][0]-0.125,portes[i][1]-0.5),0.25,1,edgecolor = 'red',facecolor ='red',fill=True))
#affichage des murs de la salle :
    axSalle.add_patch(patches.Rectangle((0,0),longueur,largeur,edgecolor = 'red',fill=False))
#affichage des obstacles rectangulaires :
    ronds=[]
    for elt in obstacles:
        if elt[0]=='rectangle':
            axSalle.add_patch(patches.Rectangle((elt[1],elt[2]),elt[3],elt[4],edgecolor = 'red',facecolor='red',fill=True))
        else:
            ronds.append(elt)
#affichage des obstacles ronds :
    axSalle.scatter([ronds[i][1] for i in range(len(ronds))],[ronds[i][2] for i in range(len(ronds))],[300*ronds[i][3] for i in range(len(ronds))], color='red')
    listeT=np.array([np.array([listeXY[0][i][0],listeXY[0][i][1]]) for i in range(len(listeXY[0]))],dtype=object)
    #print(listeXY)
#affichage des personnes au départ :
    personnes=axSalle.scatter(listeT[:, 0], listeT[:, 1], s=100*dimension, color=[0,0,0], marker=None, cmap=None, norm=None, vmin=None, vmax=None, alpha=1, linewidths=None, edgecolors=None, plotnonfinite=False, data=None)
#fonction d'animation :
    def animation(iter):
        if iter!=len(listeXY)-1:
            listeT=np.array([np.array([listeXY[iter][i][0],listeXY[iter][i][1]]) for i in range(len(listeXY[iter]))],dtype=object)
        else:
            listeT=np.array(portes,dtype=object)
        #print(listeT)
        personnes.set_offsets(listeT)
        #scat.set_array(array) pour changer les couleurs
        return personnes,
    ani=anim.FuncAnimation(fig, animation, frames=len(listeX),interval=100,repeat=False)
    plt.show()

##3D

# def nappe(longueur, largeur, portes, obstacles,n):
#     fig = plt.figure()
#     ax3d = plt.axes(projection="3d")
#     x=np.linspace(0, longueur, n)
#     y=np.linspace(0, largeur, n)
#     X,Y=np.meshgrid(x,y)
#     r=[[passageobstacleconstant(obstacles,x[i],y[j]) for i in range(n)] for j in range(n)]
#     print(r)
#     p=[[min([(x[j]-portes[i][0])**2+(y[k]-portes[i][1])**2 for i in range(len(portes))]) for j in range(n)] for k in range(n)]
#     z=[[r[i][j]+p[i][j] for i in range(n)] for j in range(n)]
#     print(z)
#     ax3d.plot_surface(x, y, z, cmap=cm.coolwarm, linewidth=0)  # Tracé d'une surface
#     plt.title("Tracé d'une surface")
#     ax3d.set_xlabel('X')
#     ax3d.set_ylabel('Y')
#     ax3d.set_zlabel('Z')
#     plt.tight_layout()
#     plt.show()

# def nappe2D(longueur, largeur, portes, obstacles, n):
#     x=np.linspace(0, longueur, n)
#     y=np.linspace(0, largeur, n)
#     for i in range(n):
#         z=[]
#         for j in range(n):
#             z.append(fcout([x[i],y[j]],portes,obstacles))
#         X=[x[i]+y[i]*2/sqrt(2) for i in range(n)]
#         Z=[z[i]+y[i]*2/sqrt(2) for i in range(n)]
#         pltplot(X,Z)
#     for j in range(n):
#         z=[]
#         for i in range(n):
#             z.append(fcout([x[i],y[j]],portes,obstacles))
#         X=[x[i]+y[i]*2/sqrt(2) for i in range(n)]
#         Z=[z[i]+y[i]*2/sqrt(2) for i in range(n)]
#         plt.plot(X,Z)
#     plt.show()

def nappe3D(longueur, largeur, portes, obstacles, n):
    x=np.linspace(0, longueur, n)
    y=np.linspace(0, largeur, n)
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    Z=[]
    for i in range(n):
        z=[]
        for j in range(n):
            z.append(fcout([x[i],y[j]],portes,obstacles,[]))
        Z.append(z)
    Z=np.array(Z)
    Z=Z.T
    X,Y=np.meshgrid(x,y)
    surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap='hot_r', linewidth=0, antialiased=False)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('distance')
    plt.show()

#nappe3D(50,20,[[0,10],[50,10]],[['rectangle',10,7.5,2,5]],200)
#nappe3D(50,20,[[0,10],[50,10]],[['cercle',10,10,3]],200)

def trajetaleatoirecomparaison(longueur, largeur,n,portes,obstacles):
    X=[uniform(0,longueur) for i in range(n)]
    Y=[uniform(0,largeur) for i in range(n)]
    for i in range(len(X)):
        while passageobstacleconstant(obstacles,X[i],Y[i])!=0:
            X[i]=X[i]+1
    dist=[0.3 for i in range(len(X))]
    listeX=[X]
    listeY=[Y]
    abs=X.copy()
    ord=Y.copy()
    absancien=X.copy()
    ordancien=Y.copy()
    marqueur=0
    m=[True for i in range(len(X))]
    while abs!=[]:
        print(marqueur)
        marqueur+=1
        trajectoires=[]
        count=0
        for i in range(len(X)):
            voisins=[[absancien[j],ordancien[j]] for j in range(len(absancien))]
            voisins.pop(i)
            if m[i]:
                trajectoires.append(gradientNDmodif(fcout,[abs[i-count],ord[i-count]],portes,obstacles,voisins,0.01+0.00005*marqueur,0.05,100))
            else:
                trajectoires.append([])
                count+=1
        abs=[]
        ord=[]
        for i in range(len(trajectoires)):
            if m[i]:
                j=0
                while j<len(trajectoires[i])-1 and ((trajectoires[i][j][0]-X[i])**2+(trajectoires[i][j][1]-Y[i])**2)<dist[i]**2:
                    j+=1
                a,o=trajectoires[i][j]
                distporte=min([(a-portes[k][0])**2+(o-portes[k][1])**2 for k in range(len(portes))])
                if distporte>2:
                    ref=True
                    for k in range(len(absancien)):#y-a-t'il quelqu'un devant à l'instant précédent :
                        if i!=k and ((a-absancien[k])**2+(o-ordancien[k])**2)<2 and fcout([absancien[k],ordancien[k]],portes,obstacles,voisins)<fcout([a,o],portes,obstacles,voisins):
                            ref=False
                    if ref:#si il n'y a personne, on avance
                        c=a+uniform(-0.01,0.01)
                        d=o+uniform(-0.01,0.01)
                        dist[i]+=0.3
                    else:#sinon, on ne bouge pas
                        c=absancien[i]
                        d=ordancien[i]
                    absancien[i]=c
                    ordancien[i]=d
                    abs.append(c)#on ajoute les coordonnées calculées si personne devant, on ne bouge pas sinon
                    ord.append(d)
                else:
                    absancien[i]=-100
                    ordancien[i]=-100
                    m[i]=False
        listeX.append(abs)
        listeY.append(ord)
    return len(listeX)

def trajetcomparaison(longueur, largeur,n,portes,obstacles):
    X=[uniform(0,longueur) for i in range(n)]
    Y=[uniform(0,largeur) for i in range(n)]
    for i in range(len(X)):
        while passageobstacleconstant(obstacles,X[i],Y[i])!=0:
            X[i]=X[i]+1
    trajectoires=[]
    for i in range(len(X)):
        print(i)
        trajectoires.append(gradientNDmodif(fcout,[X[i],Y[i]],portes,obstacles,[],0.01,0.05,10000))
    print([trajectoires[i][-1] for i in range(len(X))])
    dist=[0.3 for i in range(len(X))]
    listeX=[X]
    listeY=[Y]
    abs=[0]
    ord=[0]
    absancien=X.copy()
    ordancien=Y.copy()
    marqueur=0
    while abs!=[]:
        print(marqueur)
        marqueur+=1
        abs=[]
        ord=[]
        for i in range(len(trajectoires)):
            j=0
            while j<len(trajectoires[i])-1 and ((trajectoires[i][j][0]-X[i])**2+(trajectoires[i][j][1]-Y[i])**2)<dist[i]**2:
                j+=1
            a,o=trajectoires[i][j]
            distporte=min([(a-portes[k][0])**2+(o-portes[k][1])**2 for k in range(len(portes))])
            if distporte>2:
                ref=True
                for k in range(len(absancien)):#y-a-t'il quelqu'un devant à l'instant précédent :
                    if i!=k and ((a-absancien[k])**2+(o-ordancien[k])**2)<2 and fcout([absancien[k],ordancien[k]],portes,obstacles,[])<fcout([a,o],portes,obstacles,[]):
                        ref=False
                if ref:#si il n'y a personne, on avance
                    c=a
                    d=o
                    dist[i]+=0.3
                else:#sinon, on ne bouge pas
                    c=absancien[i]
                    d=ordancien[i]
                absancien[i]=c
                ordancien[i]=d
                abs.append(c)#on ajoute les coordonnées calculées si personne devant, on ne bouge pas sinon
                ord.append(d)
            else:
                absancien[i]=-100
                ordancien[i]=-100
        listeX.append(abs)
        listeY.append(ord)
    return len(listeX)


def temps(longueur, largeur,listen,portes,obstacles):
    X=listen
    Y=[]
    for i in range(len(listen)):
        Y.append(trajetcomparaison(longueur, largeur,listen[i],portes,obstacles)/3)
    plt.plot(X,Y)
    plt.show()

def comparaisonobstacles(longueur, largeur,listen,portes,listeobstacles):
    X=listen
    for j in range(len(listeobstacles)):
        Y=[]
        for i in range(len(listen)):
            Y.append(trajetaleatoirecomparaison(longueur, largeur,listen[i],portes,listeobstacles[j][0])/3)
        plt.plot(X,Y,label=listeobstacles[j][1])
    plt.show()

def comparaisonportes(longueur, largeur,listen,listeportes,obstacles):
    X=listen
    for j in range(len(listeportes)):
        Y=[]
        for i in range(len(listen)):
            Y.append(trajetcomparaison(longueur, largeur,listen[i],listeportes[j],obstacles)/3)
        plt.plot(X,Y)
    plt.show()

def carre(n):
    X=[10*i for i in range(1,n)]
    Y=[(10*i)**2 for i in range(1,n)]
    plt.plot(X,Y,color='y')
    plt.show()


#trajetaleatoire(50,20,2,[[0,10]],[['cercle',10,10,2]])
#trajetaleatoire(50,20,2,[[0,10]],[['cercle',10,7,2]])
#trajetaleatoire(50,20,2,[[0,10]],[])






