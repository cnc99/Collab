import statistics
import numpy
from matplotlib import pyplot as plt
import pylab

#ylego:
params = [4.995, 1e-12, 0.00135]

#For kld cost:
def scatterplot_kld():

    Data_OY =  open("plots/ylego Dataset O/kld/Dataset_OY_Opt-to-Sim_Results.txt")
    cost = []
    lateral_friction = []
    rolling_friction = []
    mass = []
    for line in Data_OY.readlines():
        cost.append(float(line.split()[3]))
        lateral_friction.append(float(line.split()[0]))
        rolling_friction.append(float(line.split()[1]))
        mass.append(float(line.split()[2]))

    plt.scatter(cost, lateral_friction, s=40, c="red", edgecolors='none', label="opt")
    plt.axhline(params[0], color='blue', linestyle='-', label="real")
    plt.legend(loc=2)
    plt.ylim(0, 6)
    plt.xlim(0, 100)
    plt.xlabel('cost')
    plt.ylabel('Lateral Friction')
    plt.title('Lateral Friction' + '\n' + 'Dataset OY')
    plt.savefig("plots/ylego Dataset O/kld/lateralFriction_datasetOY.png".format(object))
    plt.show()

    plt.scatter(cost, rolling_friction, s=40, c="red", edgecolors='none', label="opt")
    plt.axhline(params[1], color='blue', linestyle='-', label="real")
    plt.legend(loc=2)
    #plt.ylim(0, 8)
    #plt.xlim(0, 20)
    plt.xlabel('cost')
    plt.ylabel('Rolling Friction')
    plt.title('Rolling Friction' + '\n' + 'Dataset OY')
    plt.savefig("plots/ylego Dataset O/kld/rollingFriction_datasetOY.png".format(object))
    plt.show()

    plt.scatter(cost, mass, s=40, c="red", edgecolors='none', label="opt")
    plt.axhline(params[2], color='blue', linestyle='-', label="real")
    plt.legend(loc=2)
    # plt.ylim(0, 8)
    # plt.xlim(0, 20)
    plt.xlabel('cost')
    plt.ylabel('Mass')
    plt.title('Mass' + '\n' + 'Dataset OY')
    plt.savefig("plots/ylego Dataset O/kld/mass_datasetOY.png".format(object))
    plt.show()


    Data_OP = open("plots/ylego Dataset O/kld/Dataset_OP_Opt-to-Sim_Results.txt")
    cost = []
    lateral_friction = []
    rolling_friction = []
    mass = []
    for line in Data_OP.readlines():
        cost.append(float(line.split()[3]))
        lateral_friction.append(float(line.split()[0]))
        rolling_friction.append(float(line.split()[1]))
        mass.append(float(line.split()[2]))

    plt.scatter(cost, lateral_friction, s=40, c="red", edgecolors='none', label="opt")
    plt.axhline(params[0], color='blue', linestyle='-', label="real")
    plt.legend(loc=2)
    plt.ylim(0, 6)
    plt.xlim(0, 100)
    plt.xlabel('cost')
    plt.ylabel('Lateral Friction')
    plt.title('Lateral Friction' + '\n' + 'Dataset OP')
    plt.savefig("plots/ylego Dataset O/kld/lateralFriction_datasetOP.png".format(object))
    plt.show()

    plt.scatter(cost, rolling_friction, s=40, c="red", edgecolors='none', label="opt")
    plt.axhline(params[1], color='blue', linestyle='-', label="real")
    plt.legend(loc=2)
    # plt.ylim(0, 8)
    # plt.xlim(0, 20)
    plt.xlabel('cost')
    plt.ylabel('Rolling Friction')
    plt.title('Rolling Friction' + '\n' + 'Dataset OP')
    plt.savefig("plots/ylego Dataset O/kld/rollingFriction_datasetOP.png".format(object))
    plt.show()

    plt.scatter(cost, mass, s=40, c="red", edgecolors='none', label="opt")
    plt.axhline(params[2], color='blue', linestyle='-', label="real")
    plt.legend(loc=2)
    # plt.ylim(0, 8)
    # plt.xlim(0, 20)
    plt.xlabel('cost')
    plt.ylabel('Mass')
    plt.title('Mass' + '\n' + 'Dataset OP')
    plt.savefig("plots/ylego Dataset O/kld/mass_datasetOP.png".format(object))
    plt.show()

    Data_OR = open("plots/ylego Dataset O/kld/Dataset_OR_Opt-to-Sim_Results.txt")
    cost = []
    lateral_friction = []
    rolling_friction = []
    mass = []
    for line in Data_OR.readlines():
        cost.append(float(line.split()[3]))
        lateral_friction.append(float(line.split()[0]))
        rolling_friction.append(float(line.split()[1]))
        mass.append(float(line.split()[2]))

    plt.scatter(cost, lateral_friction, s=40, c="red", edgecolors='none', label="opt")
    plt.axhline(params[0], color='blue', linestyle='-', label="real")
    plt.legend(loc=2)
    plt.ylim(0, 6)
    plt.xlim(0, 100)
    plt.xlabel('cost')
    plt.ylabel('Lateral Friction')
    plt.title('Lateral Friction' + '\n' + 'Dataset OR')
    plt.savefig("plots/ylego Dataset O/kld/lateralFriction_datasetOR.png".format(object))
    plt.show()

    plt.scatter(cost, rolling_friction, s=40, c="red", edgecolors='none', label="opt")
    plt.axhline(params[1], color='blue', linestyle='-', label="real")
    plt.legend(loc=2)
    # plt.ylim(0, 8)
    # plt.xlim(0, 20)
    plt.xlabel('cost')
    plt.ylabel('Rolling Friction')
    plt.title('Rolling Friction' + '\n' + 'Dataset OR')
    plt.savefig("plots/ylego Dataset O/kld/rollingFriction_datasetOR.png".format(object))
    plt.show()

    plt.scatter(cost, mass, s=40, c="red", edgecolors='none', label="opt")
    plt.axhline(params[2], color='blue', linestyle='-', label="real")
    plt.legend(loc=2)
    # plt.ylim(0, 8)
    # plt.xlim(0, 20)
    plt.xlabel('cost')
    plt.ylabel('Mass')
    plt.title('Mass' + '\n' + 'Dataset OR')
    plt.savefig("plots/ylego Dataset O/kld/mass_datasetOR.png".format(object))
    plt.show()

    Data_OYPR = open("plots/ylego Dataset O/kld/Dataset_OYPR_Opt-to-Sim_Results.txt")
    cost = []
    lateral_friction = []
    rolling_friction = []
    mass = []
    for line in Data_OYPR.readlines():
        cost.append(float(line.split()[3]))
        lateral_friction.append(float(line.split()[0]))
        rolling_friction.append(float(line.split()[1]))
        mass.append(float(line.split()[2]))

    plt.scatter(cost, lateral_friction, s=40, c="red", edgecolors='none', label="opt")
    plt.axhline(params[0], color='blue', linestyle='-', label="real")
    plt.legend(loc=2)
    plt.ylim(0, 6)
    plt.xlim(0, 100)
    plt.xlabel('cost')
    plt.ylabel('Lateral Friction')
    plt.title('Lateral Friction' + '\n' + 'Dataset OYPR')
    plt.savefig("plots/ylego Dataset O/kld/lateralFriction_datasetOYPR.png".format(object))
    plt.show()

    plt.scatter(cost, rolling_friction, s=40, c="red", edgecolors='none', label="opt")
    plt.axhline(params[1], color='blue', linestyle='-', label="real")
    plt.legend(loc=2)
    # plt.ylim(0, 8)
    # plt.xlim(0, 20)
    plt.xlabel('cost')
    plt.ylabel('Rolling Friction')
    plt.title('Rolling Friction' + '\n' + 'Dataset OYPR')
    plt.savefig("plots/ylego Dataset O/kld/rollingFriction_datasetOYPR.png".format(object))
    plt.show()

    plt.scatter(cost, mass, s=40, c="red", edgecolors='none', label="opt")
    plt.axhline(params[2], color='blue', linestyle='-', label="real")
    plt.legend(loc=2)
    # plt.ylim(0, 8)
    # plt.xlim(0, 20)
    plt.xlabel('cost')
    plt.ylabel('Mass')
    plt.title('Mass' + '\n' + 'Dataset OYPR')
    plt.savefig("plots/ylego Dataset O/kld/mass_datasetOYPR.png".format(object))
    plt.show()





if __name__ == "__main__":
    object = "ylego"
    scatterplot_kld()