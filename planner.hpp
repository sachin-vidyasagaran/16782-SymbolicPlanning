#pragma once

using namespace std;


struct node
    {
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> state;

        double f = DBL_MAX;
        double g = DBL_MAX;
        double h = DBL_MAX;
        
        int parentIndex = -1;

        string parentNodeState = "";
    };


class PlannerClass
{
private:
    vector<GroundedAction> allGactions;
    unordered_set<Action, ActionHasher, ActionComparator> allActions;
    vector<string> allSymbols;

    Env* env;
    int numOfSym = 0;

    vector<vector<string>> combinations, permutations;
    vector<string> tempComb;

    string initNodeStr;

public:

    int numStates = 0;

    stack<GroundedAction> path;

    PlannerClass(Env* env)
    {
        this->env = env; 
    }

    void precompute();
    void getAllGactions(Action &, vector<vector<string>> &);
    void a_star();
    void getCombs(int, int);
    void getPerms(vector<string> &);
    bool goalReached(node &);
    void Astar();
    void backtrack(string &, unordered_map<string, node>&);
    double pathLength(string &, unordered_map<string, node>&, string&);
    double heurAstar(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &);
    double getHeuristic(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &);

    string hash(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &);
};