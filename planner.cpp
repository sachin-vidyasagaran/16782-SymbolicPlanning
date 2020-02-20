#include <iostream>
#include <fstream>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <cfloat>
#include <queue>
#include <stack>
#include <time.h>

#include "GroundedCondition.hpp"
#include "Condition.hpp"
#include "GroundedAction.hpp"
#include "Action.hpp"
#include "Env.hpp"
#include "planner.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int heurType = 0;   // 0 - No Heuristic, 1 - Inadmissible, 2 - Admissible

bool print_status = true;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

using namespace std;


string PlannerClass::hash(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& stateset)
{
    set<string> stringState;
    string hashedString = "";

    for(GroundedCondition gc : stateset)
    {
        stringState.insert(gc.toString());
    }
    for (auto it = stringState.begin(); it != stringState.end(); it++) 
        hashedString += *it; 
    
    return hashedString;
}

inline bool PlannerClass::goalReached(node & curNode)
{
    for(GroundedCondition gc : env->get_goal_condition())
    {   
        if(curNode.state.find(gc) == curNode.state.end())
            return 0;
    }
    return 1;
}

void PlannerClass::backtrack(string& goalNodeStr, unordered_map<string, node>& nodeInfo)
{
    string currNodeStr = goalNodeStr;
    while(currNodeStr != initNodeStr)
    {
        path.push(allGactions[nodeInfo[currNodeStr].parentIndex]);
        currNodeStr = nodeInfo[currNodeStr].parentNodeState;
    }
    return;
}

double PlannerClass::pathLength(string& goalNodeStr, unordered_map<string, node>& nodeInfo, string& h_initNodeStr)
{
    string currNodeStr = goalNodeStr;
    double pathlength = 0;
    while(currNodeStr != h_initNodeStr)
    {
        ++pathlength;
        currNodeStr = nodeInfo[currNodeStr].parentNodeState;
    }
    return pathlength;
}

double PlannerClass::getHeuristic(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& curNode)
{
    if(heurType == 0)
    {
        return 0;
    }
    else if (heurType == 1)
    {
        double heur = 0;
        for(GroundedCondition gc : env->get_goal_condition())
        {   
            if(curNode.find(gc) == curNode.end())
                ++heur;
        }
        return heur;
    }
    else if (heurType == 2)
    {
        double heur = heurAstar(curNode);
        return heur;
    }
    return 0;
}

double PlannerClass::heurAstar(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& heurNode)
{
    unordered_map<string, bool> closedList;     //closedList of bool values for each cell
    unordered_map<string, node> nodeInfo;
    priority_queue<pair<double, string>, vector<pair<double, string>>, greater<pair<double, string>>> openList;   //f-value, node index (sorted in increasing order of f-value)

    node initNode;
    initNode.state = heurNode;
    string h_initNodeStr = hash(initNode.state);
    initNode.g = 0;
    initNode.f = 0;
    initNode.h = 0;
    nodeInfo[h_initNodeStr] = initNode;
    openList.push(make_pair(initNode.f, h_initNodeStr));

    while (!openList.empty())
    {
        pair<double, string> curNodeStr = openList.top();

        openList.pop();

        // Check if current state has been expanded before
        if(closedList[curNodeStr.second] == true)
            continue;
        closedList[curNodeStr.second] = true;

        node curNode = nodeInfo[curNodeStr.second];

        // Check if current state satisfies the Goal Conditions
        if(goalReached(curNode)) 
        {
            double pathlength = pathLength(curNodeStr.second, nodeInfo, h_initNodeStr);
            return pathlength;
        }
            
        bool canDoAction = 1;
        int actCounter = -1;

        string hashedNewNode;

        // Find all the Valid Grounded Actions from this state
        for(GroundedAction ga : this->allGactions)
        {
            actCounter++;
            canDoAction = 1;

            //Find out if the action can be taken
            for(GroundedCondition gc : ga.get_preconditions())
            {   
                if(curNode.state.find(gc) == curNode.state.end())
                {
                    canDoAction = 0;
                    break;
                }
            }

            if(canDoAction)
            {
                node newNode;

                // Apply the action
                newNode.state = curNode.state;
                for(GroundedCondition eff : ga.get_effects())
                {
                    if(eff.get_truth())
                    {
                        newNode.state.insert(eff);
                    }
                }

                if(0)   // Print the state after the action has been taken
                {
                    cout<<"State After Action:\n";
                    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>::const_iterator it;
                    for(it = newNode.state.begin(); it != newNode.state.end() ; ++it)
                    {
                        cout<<*it<<",";
                    }
                    cout<<"\n";
                }

                hashedNewNode = hash(newNode.state);
                if (closedList[hashedNewNode])
                    continue;

                newNode.g = curNode.g + 1;

                double heur = 0;
                for(GroundedCondition hgc : env->get_goal_condition())
                {   
                    if(newNode.state.find(hgc) == newNode.state.end())
                        ++heur;
                }
                newNode.h = heur;

                newNode.f = newNode.g + newNode.h;

                // If the node has not been seen before (or) if the node now has a lesser cost path
                if(nodeInfo.find(hashedNewNode) == nodeInfo.end() || newNode.g < nodeInfo[hashedNewNode].g)
                {
                    newNode.parentIndex = actCounter;
                    newNode.parentNodeState = curNodeStr.second;
                    nodeInfo[hashedNewNode] = newNode;
                    openList.push(make_pair(newNode.f, hashedNewNode));
                }    
            }
            
        }
    }
    return 0;
}

void PlannerClass::Astar()
{
    unordered_map<string, bool> closedList;     //closedList of bool values for each cell
    unordered_map<string, node> nodeInfo;
    priority_queue<pair<double, string>, vector<pair<double, string>>, greater<pair<double, string>>> openList;   //f-value, node index (sorted in increasing order of f-value)

    node initNode;
    initNode.state = this->env->get_initial_condition();
    initNodeStr = hash(initNode.state);
    initNode.g = 0;
    initNode.f = 0;
    nodeInfo[initNodeStr] = initNode;
    openList.push(make_pair(initNode.f, hash(initNode.state)));

    while (!openList.empty())
    {
        pair<double, string> curNodeStr = openList.top();

        openList.pop();

        // Check if current state has been expanded before
        if(closedList[curNodeStr.second] == true)
            continue;
        closedList[curNodeStr.second] = true;
        numStates++;

        node curNode = nodeInfo[curNodeStr.second];

        // Check if current state satisfies the Goal Conditions
        if(goalReached(curNode)) 
        {
            cout<<"Number of states: "<<numStates<<"\n";
            backtrack(curNodeStr.second, nodeInfo);
            return;
        }
            

        bool canDoAction = 1;
        int actCounter = -1;

        string hashedNewNode;

        // Find all the Valid Grounded Actions from this state
        for(GroundedAction ga : this->allGactions)
        {
            actCounter++;
            canDoAction = 1;

            //Find out if the action can be taken
            for(GroundedCondition gc : ga.get_preconditions())
            {   
                if(curNode.state.find(gc) == curNode.state.end())
                {
                    canDoAction = 0;
                    break;
                }
            }

            if(canDoAction)
            {
                node newNode;

                // Apply the action
                newNode.state = curNode.state;
                for(GroundedCondition eff : ga.get_effects())
                {
                    if(eff.get_truth())
                    {
                        newNode.state.insert(eff);
                    }
                    else
                    {
                        // Remove from state
                        eff.flip_truth();
                        newNode.state.erase(newNode.state.find(eff));
                    }
                }

                if(0)   // Print the state after the action has been taken
                {
                    cout<<"State After Action:\n";
                    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>::const_iterator it;
                    for(it = newNode.state.begin(); it != newNode.state.end() ; ++it)
                    {
                        cout<<*it<<",";
                    }
                    cout<<"\n";
                }

                hashedNewNode = hash(newNode.state);
                if (closedList[hashedNewNode])
                    continue;

                newNode.g = curNode.g + 1;
                newNode.h = getHeuristic(newNode.state);
                newNode.f = newNode.g + newNode.h;

                // If the node has not been seen before (or) if the node now has a lesser cost path
                if(nodeInfo.find(hashedNewNode) == nodeInfo.end() || newNode.g < nodeInfo[hashedNewNode].g)
                {
                    newNode.parentIndex = actCounter;
                    newNode.parentNodeState = curNodeStr.second;
                    nodeInfo[hashedNewNode] = newNode;
                    openList.push(make_pair(newNode.f, hashedNewNode));
                }    
            }
            
        }
    }
}

void PlannerClass::precompute()
{
    // Store all Actions
    allActions = env->get_actions();

    // Store all Symbols as a Vetor
    vector<string> allSymbolsVect(env->get_symbols().begin(), env->get_symbols().end());
    allSymbols = allSymbolsVect;

    numOfSym = allSymbols.size();
    int numOfArgs = 0;

    int actionIndex = 0;
    
    for(Action action : allActions)
    {
        numOfArgs = action.get_args().size();
        getCombs(0, numOfArgs);
        for(int i=0 ; i<combinations.size() ; i++)
            getPerms(combinations[i]);
        
        if(0) // Print Combinations and Permutations:
        {
            cout<<"\nACTION________________\n";
            cout<<"\n ALL Combinations:\n";
            for(int i=0 ; i<combinations.size() ; i++)
            {
                for(int j=0 ; j<combinations[i].size() ; j++)
                {
                    cout<<combinations[i][j]<<" ";
                }
                cout<<"\n";
            }

            cout<<"\n ALL Permutations:\n";
            for(int i=0 ; i<permutations.size() ; i++)
            {
                for(int j=0 ; j<permutations[i].size() ; j++)
                {
                    cout<<permutations[i][j]<<" ";
                }
                cout<<"\n";
            }
        }
        
        // Send permutations to actions to get grounded actions
        getAllGactions(action, permutations);

        combinations.clear();
        tempComb.clear();
        permutations.clear();
        ++actionIndex;
    }
}

void PlannerClass::getCombs(int offset, int k)
{
    if (k == 0) 
    {
        combinations.push_back(tempComb);
        return;
    }
    for (int i = offset; i <= numOfSym - k; ++i) 
    {
        tempComb.push_back(allSymbols[i]);
        getCombs(i+1, k-1);
        tempComb.pop_back();
    }
}

void PlannerClass::getPerms(vector<string> &comb)
{
    sort(comb.begin(), comb.end()); 
    do
    { 
       permutations.push_back(comb); 
    } while (next_permutation(comb.begin(), comb.end()));
}

void PlannerClass::getAllGactions(Action &action, vector<vector<string>> &args)
{
    //Store the actions preconditions and effects
    unordered_set<Condition, ConditionHasher, ConditionComparator> action_preconds = action.get_preconditions();
    unordered_set<Condition, ConditionHasher, ConditionComparator> action_effects = action.get_effects();

    // Iterate over each permutation of arguments
    for(vector<string> a : args)
    {
        list<string> gaArgs(a.begin(), a.end());  // The specific set of symbols being used
        if(0)   // Print gaArgs
        {
            cout<<"Arguments to be inserted:\n";
            list<string>::const_iterator it;
            for(it = gaArgs.begin(); it != gaArgs.end() ; ++it)
            {
                cout<<*it<<" ";
            }
            cout<<"\n";
        }
        
        unordered_map<string, string> argMap;   // Maps the general action symbols to the corresponding grounded action symbols
        list<string> action_args = action.get_args();

        // Building the Map
        list<string>::const_iterator a_it, ga_it;
        for (a_it = action_args.begin(), ga_it = gaArgs.begin(); a_it != action_args.end(); ++a_it, ++ga_it)
            argMap[*a_it] = *ga_it;
        
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> groundedPreconds;
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> groundedEffects;

        for(Condition precond : action_preconds)
        {   
            list<string> gcArgs;
            list<string> precondArgs = precond.get_args();  // Make direct
            list<string>::const_iterator p_it;
            for(p_it = precondArgs.begin(); p_it != precondArgs.end() ; ++p_it)
            {
                if(argMap[*p_it] == "")
                {
                    gcArgs.push_back(*p_it);
                }
                else
                {
                    gcArgs.push_back(argMap[*p_it]);
                }
            }
            
            GroundedCondition gc(precond.get_predicate(), gcArgs, precond.get_truth());
            groundedPreconds.insert(gc); 
        }

        if(0)   // Print GroundedPreconditions (symbols inserted for each condition)
            {
                cout<<"Grounded Preconds:\n";
                unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>::const_iterator it;
                for(it = groundedPreconds.begin(); it != groundedPreconds.end() ; ++it)
                {
                    cout<<*it<<",";
                }
                cout<<"\n";
            }
        
        for(Condition effect : action_effects)
        {
            list<string> gcArgs;
            list<string> effectArgs = effect.get_args();  // Make direct
            list<string>::const_iterator e_it;
            for(e_it = effectArgs.begin(); e_it != effectArgs.end() ; ++e_it)
            {
                if(argMap[*e_it] == "")
                {
                    gcArgs.push_back(*e_it);
                }
                else
                {
                    gcArgs.push_back(argMap[*e_it]);
                }
                // cout<<argMap[*e_it]<<" -> "<<*e_it;
            }

            GroundedCondition gc(effect.get_predicate(), gcArgs, effect.get_truth());
            groundedEffects.insert(gc);
        }

        if(0)   // Print GroundedEffects (symbols inserted for each condition)
            {
                cout<<"Grounded Effects:\n";
                unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>::const_iterator it;
                for(it = groundedEffects.begin(); it != groundedEffects.end() ; ++it)
                {
                    cout<<*it<<",";
                }
                cout<<"\n";
            }
        GroundedAction ga(action.get_name(), gaArgs, groundedPreconds, groundedEffects);
        allGactions.push_back(ga);
    }
}

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env* create_env(char* filename)
{
    ifstream input_file(filename);
    Env* env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line == "")
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str()));  // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}

list<GroundedAction> planner(Env* env)
{
    list<GroundedAction> actions;
    
    clock_t t;
    t = clock();
    PlannerClass p(env);
    p.precompute();
    p.Astar();
    t = clock() - t;
    cout<<"Time Taken: "<<((float)t)/CLOCKS_PER_SEC<<" seconds\n";

    while(!p.path.empty())
    {
        actions.push_back(p.path.top());
        p.path.pop();
    }

    return actions;
}

int main(int argc, char* argv[])
{
    char* filename = (char*)("blocks&triangles.txt");
    if (argc > 1)
        filename = argv[1];

    cout << "Environment: " << filename << endl << endl;
    Env* env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    return 0;
}