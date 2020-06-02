#****************************************************************************
# Copyright (c) 2015 - 2016, CEA
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#*****************************************************************************

debug_level=0
ite=0
def interprete_file(file,fileout,traitement_special):
    ''' lit le fichier file et le preprocess, la sortie est dans le ficher fileout'''
    try:
    # or codecs.open on Python <= 2.5
    # or io.open on Python > 2.5 and <= 2.7
       filedata = open(file, encoding='UTF-8').read() 
    except:
       print("Error! Convert",file,"to UTF-8 format.")
       exit(1)
    # on recupere la sortie standard pour le mettre en stderr
    import sys,io
    s = io.StringIO()
    #sys.stdout= s
    #print 'lecture de',file

    in0=read_file(file)
    list_macro={}
    list_vars={}
    str_in=interprete_string(in0,list_macro,list_vars)
    if (str_in.find('#P')>=0):
        print("en sortie il reste des #P !!!!")
        exit(1)
    #print 'ecriture de',fileout
    if traitement_special:
        # str_in=str_in.replace('\n\n','\n')
        j=0
        pos=0
        # on cherche la 8 ligne
        insert_at=7
        if str_in.find('CEA')>0:
            insert_at=21

        while (j<insert_at):
            j+=1
            pos=str_in.find('\n',pos)+1
            pass
        str_in=str_in[:pos]+'//\n// WARNING: DO NOT EDIT THIS FILE! Only edit the template file '+file+'\n//\n'+str_in[pos:]
        pass

    sys.stderr.write(s.getvalue())
    sys.stdout = sys.__stdout__
    if (fileout!=sys.stdout):
        f3=open(fileout,'w')
        f3.write(''.join(str_in))
        f3.close()
    else:
        print(str_in)
    pass




def interprete_string(str_in,list_macro,list_vars):
    ''' le preprocesseur modifie str_in tant qu il trouve un pattern ou une variable'''
    import sys
    global ite,debug_level

    list_pattern={}
    for pat in "macro","usemacro","set","unset","if","include","foreach","error","comment","add":
        # Change of behavior of exec() in Python3:
        #exec("patt=pattern_"+pat+"()", context)
        # Better to use this:
        func_name = "pattern_%s" % pat
        p = getattr(sys.modules[__name__], func_name)()
        list_pattern[p.get_pattern()] = p 
        pass

    pattern,position=cherche_pattern(str_in,list_pattern,list_vars)

    while (position>=0):
        if (debug_level): print("ppp ",position,pattern.get_pattern())
        str_in,list_macro,list_vars=pattern.traite_pattern(position,str_in,list_macro,list_vars)
        pattern,position=cherche_pattern(str_in,list_pattern,list_vars,position)
        ite+=1
        if debug_level>0:
            print("ITE",ite," write to file debug.ITE")
            f=open('debug.'+str(ite),'w')
            f.write(str_in)
            f.close()
            pass
        pass
    return str_in
    pass


def cherche_pattern(str_in,list_pattern,list_vars,debut=0):
    ''' recherche la position du premier pattern dans str_in, renvoit le pattern et la position'''
    position=len(str_in)
    #debut=0
    mot_trouve=None
    list_mot=list(list_pattern.keys())
    for mot in list(list_vars.keys()):
        if (len(list_vars[mot])>0):
            list_mot.append(mot)
            pass
        pass
    if (debug_level>2): print("pattertns ",list_mot)
    for mot in list_mot:
        # print "A",mot,str_in[:position]
        n=str_in[:position].find(mot,debut)
        # print n
        if (n>=0):
            position=n
            mot_trouve=mot
            pass
        pass
    if (mot_trouve):
        if (mot_trouve in list(list_pattern.keys())):
            pattern=list_pattern[mot_trouve]
        else:
            pattern=list_vars[mot_trouve][-1]
            pass
        pass
    else:
        pattern=None
        position=-1
        pass
    return pattern,position

def read_file(file):
    ''' lit le fichier et le convertit en chaine de caractere'''
    f=open(file,'r')
    strin=''.join(f.readlines())
    if (strin.find('#Pset (')>0):
        print("attention votre fichier",file,"contient '#Pset (' au lieu de '#Pset(', on substitue")
        strin=strin.replace( '#Pset (', '#Pset(')
        pass
    return strin
def find_fermant(in0,ouvrant='(',fermant=')'):
    ''' cherche la position du bloc fermant en tenant compte des blos ouvrant fermant'''
    if in0[:len(ouvrant)]!=ouvrant:
        raise Exception("Pb :"+ouvrant+' pas le debut de '+in0[:20])
    fin=0
    n=1
    nmax=len(in0)
    while (n!=0)and(fin<nmax):
        fin+=1
        #print in0[fin],fin,n
        if in0[fin:fin+len(fermant)]==fermant:
            n-=1
        elif in0[fin:fin+len(ouvrant)]==ouvrant:
            n+=1
            pass
        pass
    if fin>=nmax:
        print("  on n a pas trouve ",fermant,"....")
        return -1
    else:
        return fin
    pass

class pattern_base:
    ''' tous les pattern heritent de cette classe et doivent definir analyse(in2,list_macro,list_vars) ainsi que les attributs de classe pattern et end_pattern    '''
    bloc_fin_with_varname=0
    def get_name(self):
        return self.name
    def get_pattern(self):
        return self.pattern
    def get_end_pattern(self):
        return self.end_pattern
    def decoupe(self,str_in,position):
        ''' renvoie 3 chaines de caractere :
        -- ce qui est avant le pattern
        -- le bloc du pattern
        -- ce qui est apres le pattern'''
        pate=self.get_pattern()
        end_pate=self.get_end_pattern()
        nb_bloc=1
        in2=str_in[position:]
        if (end_pate==')'):
            nb_bloc=self.nb_bloc
            index2=len(pate)-2
            prov=in2
            for i in range (nb_bloc):
                prov=in2[index2+1:]
                # print "PPP",prov
                index2=index2+1+find_fermant(prov)
                pass
            in1=str_in[:position]
            in3=in2[index2+1:]
            in2=in2[len(pate)-1:index2+1]
            # print "in2",in2
            # on recupere ()
        else:
            if (self.bloc_fin_with_varname==1):
                end_pate+='('+in2[len(pate):in2.find('(')].strip()+')'
                index2=in2.find(end_pate)
            else:
                index2=find_fermant(in2,pate,end_pate)
                pass
            if (index2<0):
                print("uu", end_pate, 'not_found in ' , in2)
                raise Exception(end_pate+' not_found in '+in2[:100]+" .....voir + haut")
            in1=str_in[:position]
            in3=in2[index2+len(end_pate):]
            in2=in2[:index2]
            if (in2.find(pate)!=0):
                print(in2,pate)
                raise Exception(in2[100:]+" ne commence par par "+pate)
            in2=in2[len(pate):]
            pass
        # si le caractere suivant la fin de l'instruction est NL on le retire
        if (len(in3)>1):
            if (in3[0]=='\n'): in3=in3[1:]

        return in1,in2,in3
    def traite_pattern(self,position,str_in,list_macro,list_vars):
        ''' simple appel a decoupe puis a analyse, avant de tout regrouper'''
        in1,in2,in3=self.decoupe(str_in,position)
        in2mod=self.analyse(in2,list_macro,list_vars)
        str_out=in1+in2mod+in3
        return str_out,list_macro,list_vars

    pass

class pattern_error(pattern_base):
    ''' usage: #Perror(message) '''
    pattern='#Perror('
    end_pattern=')'
    nb_bloc=1
    def analyse(self,in2,list_macro,list_vars):
        raise Exception("#Perror found " +in2)
    pass
class pattern_macro(pattern_base):
    ''' usage: #Pmacro name(arg1...)BLOC#endmacro(name)
       permet de definir la macro name '''
    pattern='#Pmacro'
    end_pattern='#Pendmacro'
    bloc_fin_with_varname=1
    def analyse(self,in2,list_macro,list_vars):
        nom_macro=in2.split('(')[0].strip()
        if (macro not in list(list_macro.keys())): list_macro[nom_macro]=[]

        list_macro[nom_macro].append(macro(nom_macro,in2))
        return ""
    pass
class pattern_foreach(pattern_base):
    ''' usage: #Pforeach name (val1 val2)BLOC#Pendforeach(name)
       permet de definir une boucle. Separateur espace. val1 val2 interprete avant '''
    pattern='#Pforeach'
    end_pattern='#Pendforeach'
    bloc_fin_with_varname=1
    def analyse(self,in2,list_macro,list_vars):
        index1=in2.index('(')
        varname=in2[:index1].strip()
        suite=in2[index1:]
        index2=find_fermant(suite)
        values=suite[1:index2]
        values=interprete_string(values,list_macro,list_vars)
        values=values.split()
        bloc=suite[index2+1:]
        out=""
        for val in values:
            out+="#Pset("+varname+" "+val+")"+bloc+"#Punset("+varname+")"
            pass
        return out
    pass
class pattern_usemacro(pattern_base):
    ''' usage: #Pusemacro(name)(arg1,arg... )
       cette ligne sera remplace par le BLOC de la macro, en subsituant les arguments'''
    pattern='#Pusemacro('
    end_pattern=')'
    nb_bloc=2
    def analyse(self,in2,list_macro,list_vars):
        # print "RRR",in2
        bloc=in2
        iuse=0
        i1=iuse+find_fermant(in2[iuse:])
        #i2=i1+1+find_fermant(in2[i1+1:])
        nom_macro=in2[iuse+1:i1]
        args=in2[i1+2:-1]
        ##
        #bloc=in0[iuse0:i2+1]
        macr=list_macro[nom_macro][-1]
        # print "ARRR",args
        bloc=macr.traite(args)
        # si le caractere suivant la fin de l'instruction est NL on le retire
        if (bloc[0]=='\n'): bloc=bloc[1:]
        # print "iii ",bloc
        return bloc
    pass

class pattern_set(pattern_base):
    ''' usage: #Pset( var val1..val2)
          permet d enregistrer var dans la liste des variables connues
          val1..val2 est interprete avant le stockage
          Remarque les variables s empilent
          set (var 1)
          set(var 2)
          unset(var)
          var vaudra de nouveau 1
          '''
    pattern='#Pset('
    end_pattern=')'
    nb_bloc=1
    def analyse(self,in2,list_macro,list_vars):
        index=in2.find(' ')
        name_var=in2[1:index]
        value=in2[index+1:-1]
        value=interprete_string(value,list_macro,list_vars)
        # print "uu",name_var,"value",value
        if (name_var not in list(list_vars.keys())): list_vars[name_var]=[]
        list_vars[name_var].append(pattern_value(name_var,value))

        return ""
    pass
class pattern_add(pattern_base):
    ''' usage: #Padd( var_in var_out val)
          permet d incrementer une variable de type entier
          var_out = var_in + value 
          '''
    pattern='#Padd('
    end_pattern=')'
    nb_bloc=1
    def analyse(self,in2,list_macro,list_vars):
        index=in2.find(' ')
        params = in2.split('(')[1].split(')')
        params = list(filter(lambda x: x!="",params))
        params = params[0].strip().split() 

        if len(params)!=3:
          raise Exception("Padd : On attendait 3 parametres. On a lu les parametres : "+params)

        name_var_in  = params[0]
        name_var_out = params[1]
        name_op      = params[2]

        if (name_var_in not in list_vars.keys()): 
          raise Exception("Padd : La variable a incrementer "+name_var_in+" n'est pas definie.")
          
        value_op = interprete_string(name_op,list_macro,list_vars)

        value_in = list_vars[name_var_in][0].get_value()
        if len(value_in.split())!=1:
          raise Exception("Padd : on attendait qu'une seule valeur a incrementer. On a lu les valeurs "+value_in)
        else:
          value_in = value_in.split()[0]

        # On verifie que l'on a le droit de faire un cast
        n1 = 0
        n2 = 0
        try:
          n1 = int(value_in)
        except ValueError:
         raise Exception("Padd : le parametre a incrementer doit etre compatible avec un entier. On a lu "+value_to_incr)
        try:
          n2 = int(value_op)
        except ValueError:
          raise Exception("Padd : l'operande doit etre compatible avec un entier. On a lu "+operand)

        list_vars[name_var_out]=[]
        value_out = str(n1+n2)
        list_vars[name_var_out].append(pattern_value(name_var_out,value_out))
        return ""
    pass
class pattern_unset(pattern_base):
    ''' usage: #Punset(var)
    permet d effacer la derniere valeur de  var dans la liste des variables connues
    Remarque les variables s empilent
    #Pset (var 1)
          #Pset(var 2)
          #Punset(var)
          -> var vaudra de nouveau 1
          #Punset(var)
          -> var n est plus compris
          '''
    pattern='#Punset('
    end_pattern=')'
    nb_bloc=1
    def analyse(self,in2,list_macro,list_vars):
        #index=in2.find(' ')
        name_var=in2[1:-1].strip()
        # print "uu",name_var,"value",value
        if (name_var not in list(list_vars.keys())):
            print("liste variables", list(list_vars.keys()))
            raise Exception(""+name_var+" non defini et on essaye de la retirer")
        if(len(list_vars[name_var])==0):
            raise Exception(name_var+" plus defini et on essaye de la retirer")
        list_vars[name_var]=list_vars[name_var][:-1]
        #print "mmm" ,list_vars[name_var]
        return ""
    pass
class pattern_comment(pattern_base):
    '''usage: #Pcomment coment ....
              ce qui suit jusqu a la fin de la igne est ignore'''
    pattern='#Pcomment'
    end_pattern='\n'
    nb_bloc=1
    def analyse(self,in2,list_macro,list_vars):
        # print "comment",in2
        return ""
    pass
class pattern_include(pattern_base):
    '''usage: #Pinclude(file)
             permet d inclure le fichier  file (variabel interprete avant)
             a la place de l instruction'''
    pattern='#Pinclude('
    end_pattern=')'
    nb_bloc=1
    def analyse(self,in2,list_macro,list_vars):
        file=in2[1:-1]
        file=interprete_string(file,list_macro,list_vars)
        return read_file(file)

    pass

class pattern_if(pattern_base):
    '''usage: #Pif(condition1)  bloc [ #Pelif(condition2) bloc2 ]  [ #Pelse bloc3 ] #Pendif
          les conditions sont interpretes puis evalues par le python
          Ex:  #Pset (__titi__ 1)
               #Pif (__titi__ == 1)
                 ...
               #Pendif

          '''
    pattern='#Pif'
    end_pattern='#Pendif'
    def traite(self,in2,motsepa):
        ''' recherche du premier motsepa sans tenir compte des bloc #Pif .... #Pendif'''
        nargs2_p=in2.split(motsepa)
        nargs2=[]
        motc=""
        np=0
        ouvrant='#Pif'
        fermant='#Pendif'
        for mot in nargs2_p:


            for i in range(len(mot)):
                if (mot[i:i+len(ouvrant)]==ouvrant):
                    np+=1
                elif  (mot[i:i+len(fermant)]==fermant):
                    np-=1
                    pass
                pass
            if (motc==""):
                motc=mot
            else:
                motc=motc+motsepa+mot
                pass

            if (np==0):
                nargs2.append(motc)
                motc=""
                pass
            pass
        if (np!=0):
            print(nargs2,nargs2_p)
            print(np,motsepa,"pb avec in2",in2)
            raise Exception("echec du decoupage du if ")
        return nargs2

    def analyse(self,in2,list_macro,list_vars):
        # print "if in2:::", in2
        # print "finin2"
        index1=in2.find('(')

        name_var=in2[:index1].strip()
        condition=in2[index1:]
        fin_condition=find_fermant(condition)
        condition=" "+condition[1:fin_condition]
        # print condition
        condition=interprete_string(condition,list_macro,list_vars)
        # print "CCCC", condition,list_vars.keys()
        testcond=eval(condition)
        # print "COND",condition,testcond
        marq_else='#Pelse'
        marq_elif='#Pelif'
        res_elif=self.traite(in2,marq_elif)
        res_else=self.traite(in2,marq_else)
        fin_vrai=len(in2)
        if (len(res_elif)>1):
            bloc_false="#Pif"+marq_elif.join(res_elif[1:])+"#Pendif"
            fin_vrai=len(res_elif[0])
        elif (len(res_else)>1):
            bloc_false=res_else[1]
            fin_vrai=len(res_else[0])
        else:
            bloc_false=""
            pass

        bloc_true=in2[index1+fin_condition+1:fin_vrai]
        # si le caractere suivant la fin de l'instruction est NL on le retire
        if (bloc_true[0]=='\n'): bloc_true=bloc_true[1:]
        if (len(bloc_false)>0) and (bloc_false[0]=='\n'): bloc_false=bloc_false[1:]
        if (debug_level>1):
            print("TEST" ,condition,testcond)
            print("VRAI",bloc_true)
            print("FAUX",bloc_false)
            pass
        if testcond:

            return bloc_true
        else:
            return bloc_false
        pass

    pass

class macro:
    '''classe de stockage interne
          stocke le bloc d une macro '''
    def __init__(self,name,str):
        self.name=name
        # print str
        self.str=str[str.find(name)+len(name):]
        i1=find_fermant(self.str)

        self.args=self.str[1:i1]
        self.bloc=self.str[i1+1:]
        # print "MACRO",self.name,self.bloc
        pass
    def traite(self,args):
        ''' fait la substitution des args de la macro par les arguments passes'''
        nargs=self.args.split(',')

        nargs2_p=args.split(',')
        nargs2=[]
        motc=""
        np=0
        for mot in nargs2_p:
            for i in range(len(mot)):
                if (mot[i]=='(') or (mot[i]=='['):
                    np+=1
                elif  (mot[i]==')') or (mot[i]==']'):
                    np-=1
                    pass
                pass
            if (motc==""):
                motc=mot
            else:
                motc=motc+","+mot
                pass

            if (np==0):
                nargs2.append(motc)
                motc=""
                pass
            pass
        if (np!=0):
            raise Exception("pb avec args"+args)

        # print "ARRR", nargs2_p,nargs2
        bloc2=self.bloc
        if (len(nargs)!=len(nargs2)):
            raise  Exception("Pb a l'appel de la macro: "+self.name+" pas le bon nombre d arguments "+len(nargs2)+" au lieu de "+len( nargs))
        for k in range(len(nargs)):
            # print k,nargs[k],nargs2[k]
            bloc2=bloc2.replace(nargs[k].strip(),nargs2[k])
            pass
        # print nargs
        # print "MM ",bloc2

        return bloc2
    pass
class  pattern_value(pattern_base):
    '''classe de stockage interne
          stocke les valeurs des variables'''
    def get_pattern(self):
        return self.name

    def __init__(self,name,str):
        self.name=name
        self.str=str
        pass
    def decoupe(self,str_in,position):
        in1=str_in[:position]
        position2=position+len(self.name)
        in2=str_in[position:position2]
        in3=str_in[position2:]
        return in1,in2,in3
    def analyse(self,in2,list_macro,list_vars):
        if (in2!=self.name):
            raise Exception("PB"+in2+" "+self.name)
        return self.str
    def get_value(self):
        return self.str
    pass


if  __name__ == '__main__':
    import sys
    #sys.stderr.write("usage: python "+sys.argv[0]+"[ -traitement_entete_trio][--debug_level==level] [-help] file_to_preprocess\n")
    argv=sys.argv[1:]
    traitement_entete=0
    if len(argv)==0:
        interprete_file('test2','es3')
    else:
        if (argv[0][:len("-traitement_entete_trio")]=="-traitement_entete_trio"):
            traitement_entete=1
            argv=argv[1:]
            pass
        if (argv[0][:14]=="--debug_level="):
            debug_level=eval(argv[0][14:])
            argv=argv[1:]
            pass
        if (argv[0]=="-help"):
            help(__name__)
        else:
            if len(argv)==1:
                interprete_file(argv[0],'es3',traitement_entete)
            else:
                if (argv[0]==argv[1]):
                    raise Exception("meme fichier ne entree et en sortie")
                interprete_file(argv[0],argv[1],traitement_entete)
                #interprete_file(file,sys.stdout)
                pass
            pass
        pass
    pass
