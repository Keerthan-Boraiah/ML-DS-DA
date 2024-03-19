from hanabi_learning_environment import rl_env
from simple_agent import MyAgent as OurAgent
import platform
import numpy

num_players=4 # can change this as required to 2,3,4 or 5.
environment=rl_env.make('Hanabi-Full', num_players=num_players)
if platform.system()=="Windows":
    # We're on a Windows OS.
    # A temporary work-around to fix the problem that it seems on Windows, the random seed always shuffles the deck exactly the same way.
    import random
    for i in range(random.randint(1,100)):
        observations = environment.reset()


##GENETIC ALGORITHM##
###########################################################################################################################################

#SELECT PARENTS
def select_parent(pop, fitness, parents_split_input):
    parents = numpy.empty((parents_split_input, pop.shape[1]))
    for parent_num in range(parents_split_input+1):
        if parent_num == parents_split_input:
            break
        else:
            max_fitness = numpy.where(fitness == numpy.max(fitness))
            max_fitness = max_fitness[0][0]
            parents[parent_num, :] = pop[max_fitness, :]
    return parents

#SELECT OFFSPRING
def crossover_child(parents, offspring_size):
    """
    DESCRIPTION:- Python code that follows the line
    offspring = numpy.empty(offspring_size)
    crossover_point = numpy.int(offspring_size[1]/2)
    AUTHOR:- Ahmed Gad
    DATE:- July 18, 2018
    URL:- https://towardsdatascience.com/genetic-algorithm-implementation-in-python-5ab67bb124a6
    REASON:- Crossover takes place between two parents. Usually, it is at the center
    """
    offspring = numpy.empty(offspring_size)
    crossover_point = numpy.int(offspring_size[1]/2)
 
    for i in range(offspring_size[0]+1):
        if i == offspring_size[0]:
            break
        else:
            offspring[i, 0:crossover_point] = parents[i%parents.shape[0], 0:crossover_point]
            offspring[i, crossover_point:] = parents[(i+1)%parents.shape[0], crossover_point:]
    return offspring

#MUTATION
def mutation_child_parent(offspring):
    for i in range(offspring.shape[0]+1):
        if i == offspring.shape[0]:
            break
        else:
            """
            DESCRIPTION:- Python code that follows the line 
            random_value = numpy.random.randint(-1.0, 1.0, 1) 
            offspring[i, 4] = offspring[i, 4] + random_value
            AUTHOR:- Ahmed Gad
            DATE:- July 18, 2018
            URL:- https://towardsdatascience.com/genetic-algorithm-implementation-in-python-5ab67bb124a6
            REASON:- It mutates change in the single gene in each offspring randomly
            and inside for loop if the condition does not break the random value is added to the gene
            """
            random_value = numpy.random.randint(-1.0, 1.0, 1) 
            offspring[i, 4] = offspring[i, 4] + random_value
    return offspring

##########################################################################################################################################
##

fitness = []
fit = []
"""
DESCRIPTION:- Python code that follows the line
total_population = numpy.array([[0,2,5,1,1],[1,2,5,3,6],[0,2,6,3,5],[3,2,1,5,6]])
AUTHOR:- Myself
REFERENCE:- Towardsdatasciene
URL:- ttps://towardsdatascience.com/genetic-algorithm-implementation-in-python-5ab67bb124a6
REASON:- The reason why i included the 5 length gene is because I had some trouble with first, for generating a
random initial population let's say 2 genes, and a genetic algorithm to move up the gene value to 3,4,5 to get a good fitness value. 
So, I did my best to at least work with only the 5th gene which was giving a good score. 
So I add only 5 gene initial populations instead of randomly generating the genes.
THESE 5 chromosomes below have good scores and bad scores, so genetic algorithm will learn and give me only good chromosomes.
"""
total_population = numpy.array([[0,2,5,1,1],[1,2,5,3,6],[0,2,6,3,5],[3,2,1,5,6]])

global Generation,parent_split,weights,number_of_cromosomes
Generations = 5
parent_split = 2

weights = 5
number_of_cromosomes = 4


for i in total_population:
    observations = environment.reset() # starts a new game, i.e. shuffles and deals the cards.
    # Build the team of players - each programmed with the same agent logic ("Mirror Mode")
    # Even though they are the same program logic for each player, they cannot exchange information
    # between each other, for example to see their own hand.
    agents = [OurAgent({'players': num_players},list(i)) for _ in range(num_players)] 
    done = False
    episode_reward = 0
    while not done:
        for agent_id, agent in enumerate(agents):
            observation = observations['player_observations'][agent_id]
            action = agent.act(observation)
            if observation['current_player'] == agent_id:
                assert action is not None   
                current_player_action = action
                print("Player",agent_id,"to play")
                print("Player",agent_id,"View of cards",observation["observed_hands"])
                print("Fireworks",observation["fireworks"])
                print("Player",agent_id,"chose action",action)
                print()
            else:
                assert action is None

        # Make an environment step.
        observations, reward, done, unused_info = environment.step(current_player_action)
        if reward<0:
            reward=0 # we're changing the rules so that losing all lives does not result in the score being zeroed.
        episode_reward += reward

    print("Game over.  Fireworks",observation["fireworks"],"Score=",episode_reward)
    fitness.append(episode_reward)
    fit.append(episode_reward)

for generation in range(Generations):
    print('The Generation is:- ',generation)

    parents = select_parent(total_population, fitness, parent_split)
    offspring_crossover = crossover_child(parents,offspring_size=(number_of_cromosomes-parents.shape[0], weights))   
    offspring_mutation = mutation_child_parent(offspring_crossover)
    total_population[0:parents.shape[0], :] = parents
    total_population[parents.shape[0]:, :] = offspring_mutation

    print(total_population)


################################################################################################################
#FINAL USE OF CHROMOSOME
print('The Final Cromosome from GENETIC ALGORITHM IS :- ',total_population[-1],"With Score:- ",max(fit))
################################################################################################################