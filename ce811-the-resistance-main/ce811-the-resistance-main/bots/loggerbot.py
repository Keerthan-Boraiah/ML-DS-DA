from player import Bot
from game import State
import random

# run this with python competition.py 10000 bots/intermediates.py bots/loggerbot.py  
# Then check logs/loggerbot.log   Delete that file before running though
from player import Bot
from game import State
import random

class LoggerBot(Bot):
    def select(self, players, count):
        return [self] + random.sample(self.others(), count - 1)

    def vote(self, team):
        return True

    def sabotage(self):
        return True

    def mission_total_suspect_count(self, team):
        count = 0
        for p in team:
            count = count + self.failed_missions_been_on[p]
        return min(count, 5)

    def onVoteComplete(self, votes):
        total_suspect_count = self.mission_total_suspect_count(self.game.team)
        i = 0
        for z in self.game.players:
            self.num_missions_voted_up_with_total_suspect_count[z][total_suspect_count] += votes[i]
            i += 1

        j = 0
        for z in self.game.players:
            self.num_missions_voted_down_with_total_suspect_count[z][total_suspect_count] += not votes[j]
            j += 1
        for z in self.game.players:
            self.training_feature_vectors[z].append(
            [self.game.turn, self.game.tries, z.index, z.name, self.missions_been_on[z],
            self.failed_missions_been_on[z]] + self.num_missions_voted_up_with_total_suspect_count[z] +
            self.num_missions_voted_down_with_total_suspect_count[z])

    def onGameRevealed(self, players, spies):
        self.failed_missions_been_on = {}
        for z in players:
            self.failed_missions_been_on[z] = 0

        self.missions_been_on = {}
        for z in players:
            self.missions_been_on[z] = 0

        self.num_missions_voted_up_with_total_suspect_count = {}
        for z in players:
            self.num_missions_voted_up_with_total_suspect_count[z] = [0, 0, 0, 0, 0, 0]

        self.num_missions_voted_down_with_total_suspect_count = {}
        for z in players:
            self.num_missions_voted_down_with_total_suspect_count[z] = [0, 0, 0, 0, 0, 0]

        self.training_feature_vectors = {}
        for z in players:
            self.training_feature_vectors[z] = []

    def onMissionComplete(self, num_sabotages):
        if num_sabotages:
            for z in self.game.team:
                self.failed_missions_been_on[z] += 1
                self.missions_been_on[z] += 1
        else:
            for z in self.game.team:
                self.missions_been_on[z] += 1

    def onGameComplete(self, win, spies):
        for player_number in range(len(self.game.players)):
            player = self.game.players[player_number]
            spy = player in spies
        feature_vectors = self.training_feature_vectors[player]
        for z in feature_vectors:
            z.append(1 if spy else 0)
        self.log.debug(','.join(map(str,z)))