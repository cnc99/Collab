


def sqdlist(l):
    i=0
    while(i<len(l)):
        l[i]=(l[i])**2
        i=i+1
    return l

#trace of a matrix
def trace(m):
    if len(m)!= len(m[0]):
        print('matrix is non-square')
    else:
        i=0
        a=0
        while i<len(m[0]):
            a=a+m[i][i]
            i=i+1
        return a

#create identity matrix
def id(k):
   i=0
   id=[]
   while i<k:
       j=0
       row=[]
       while j<k:
           if j==i:
               row=row+[1]
           else:
               row=row+[0]
           j=j+1
       id=id+[row]
       i=i+1
   return id

