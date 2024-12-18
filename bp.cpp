/* 046267 Computer Architecture - HW #1                                 */
/* This file should hold your implementation of the predictor simulator */

//#define PRINT_AUX

#ifdef PRINT_AUX
#include <iostream>
#include <iomanip> // for std::setw
#endif

#include "bp_api.h"
#include <cmath>   // std::log2()
#include <string>  // std::sting
#include <cstdint> // ? Fixed width integer types (since C++11)

// ---*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-- //
// ? Aux Functions

// assumes function parmaters are valid
uint32_t get_tag(uint32_t pc, uint8_t tag_length, uint8_t index_length, uint8_t alignment = 2);
// assumes function parmaters are valid
uint32_t get_index(uint32_t pc, uint8_t index_length, uint8_t alignment = 2);

// ---*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-- //
// ? definitions

// the `: uint8_t` ensures that the enum integral type is uint8_t (for types to be the same)
enum fsm_state : uint8_t
{
	SNT = 0,
	WNT = 1,
	WT = 2,
	ST = 3
};

class BranchPredictor
{
public:
	BranchPredictor() = default; // initializes every type to 0
	~BranchPredictor();

	void update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst);

	/*
	 ? Useful
	 */
	uint32_t get_BTB_entry_branch_pc(uint32_t idx) { return __btb[idx].__branch_pc; }
	uint32_t get_BTB_entry_target_pc(uint32_t idx) { return __btb[idx].__target_pc; }
	void set_BTB_entry_branch_pc(uint32_t idx, uint32_t pc) { __btb[idx].__branch_pc = pc; }
	void set_BTB_entry_target_pc(uint32_t idx, uint32_t pc) { __btb[idx].__target_pc = pc; }
	bool is_BTB_entry_valid(uint32_t idx) { return __btb[idx].__valid; }
	void set_BTB_as_valid(uint32_t idx) { __btb[idx].__valid = true; }
	void set_BTB_as_invalid(uint32_t idx) { __btb[idx].__valid = false; }
	bool is_BTB_decisive() const { return (this->get_tag_size() + static_cast<uint8_t>(std::log2(this->get_btb_size())) + 2) == 32; }
	bool get_BTB_prediction(uint32_t pc);
	uint32_t get_fsm_index(uint32_t pc);

	void updateFSMState(fsm_state &current_state, bool taken)
	{
		if (taken)
		{
			if (current_state < ST)
			{
				current_state = static_cast<fsm_state>(static_cast<int>(current_state) + 1);
			}
		}
		else
		{
			if (current_state > SNT)
			{
				current_state = static_cast<fsm_state>(static_cast<int>(current_state) - 1);
			}
		}
	}

	/*
	 ? Init values Getters and Setters
	 */
	inline unsigned get_btb_size() const { return __btb_size; }
	inline unsigned get_history_size() const { return __history_size; }
	inline unsigned get_tag_size() const { return __tag_size; }
	inline unsigned get_fsm_default_state() const { return __fsm_default_state; }
	inline bool is_global_history() const { return __global_history; }
	inline bool is_global_table() const { return __global_table; }
	inline int get_share() const { return __share; }

	bool set_btb_size(unsigned btb_size)
	{
		__btb_size = btb_size;
		return true;
	}
	bool set_history_size(unsigned history_size)
	{
		__history_size = history_size;
		return true;
	}
	bool set_tag_size(unsigned tag_size)
	{
		__tag_size = tag_size;
		return true;
	}
	bool set_fsm_default_state(unsigned fsm_default_state)
	{
		__fsm_default_state = (fsm_state)fsm_default_state;
		return true;
	}
	bool set_global_history(bool global_history)
	{
		__global_history = global_history;
		return true;
	}
	bool set_global_table(bool global_table)
	{
		__global_table = global_table;
		return true;
	}
	bool set_share(int share)
	{
		__share = share;
		return true;
	}

	/*
	 ? Predictor operations
	 */
	bool create_BTB();
	bool create_history();
	bool create_fsm_table();

	/*
	 ? Stats Getters and Setters
	 */

	inline SIM_stats &get_stats() { return __stats; }
	inline unsigned &Stats_flush_num() { return __stats.flush_num; }
	inline unsigned &Stats_br_num() { return __stats.br_num; }
	inline unsigned &Stats_size() { return __stats.size; }

private:
	/*
	 ? Init values
	 */
	// valid values: 1,2,4,8,16,32
	unsigned __btb_size;
	// valid bit number: 1-8
	unsigned __history_size;
	// valid bit number: 0-( 32-log2[__btb_size]-2 )
	unsigned __tag_size;
	// valid values: enum fsm_state
	fsm_state __fsm_default_state;
	// true for global history
	bool __global_history;
	// true for global fsm
	bool __global_table;
	// using Lshare or Gshare, only relevant for global fsm (__global_table==true)
	// not_using_share == 0
	// using_share_lsb == 1 : XOR
	// using_share_mid == 2 : XOR
	int __share;

	/*
	 ? BTB entry
	 */
	struct BTB_entry
	{
		uint32_t __branch_pc;
		uint32_t __target_pc;
		bool __valid;
	};
	BTB_entry *__btb;

	/*
	 ? BTB History - local/global
	 */
	struct BTB_history
	{
		uint8_t __history_bit_map; // __history_bit_map contains the n bits for history (assuming __history_size is 8 bits max)
	};
	BTB_history *__history;

	/*
	 ? Bimodal fsm - local/global
	 */
	struct Bimodal_FSM
	{
		fsm_state *__bimodal_FSM_state; // 2 bits, stored in a byte for simplicity
	};
	Bimodal_FSM *__fsm;

	/*
	 ? Stats
	 */
	SIM_stats __stats;

	// ---*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-- //

#ifdef PRINT_AUX
public:
	void printBTBTable()
	{

		// Print the header row
		std::cout << std::setw(12) << "Index"
				  << std::setw(15) << "Tag"
				  << std::setw(15) << "Target PC"
				  << std::setw(10) << "Valid"
				  << std::endl;

		std::cout << std::string(52, '-') << std::endl;

		// Print each BTB entry
		for (size_t i = 0; i < __btb_size; ++i)
		{
			std::cout << std::setw(12) << i
					  << std::setw(15) << get_tag(__btb[i].__branch_pc, this->get_tag_size(), (uint8_t) std::log2(this->get_btb_size()))
					  << std::setw(15) << __btb[i].__target_pc
					  << std::setw(10) << (__btb[i].__valid ? "true" : "false")
					  << std::endl;
		}
	}

	void printHistory()
	{
		if (is_global_history())
		{
			std::cout << std::setw(12) << "Index"
					  << std::setw(15) << "History"
					  << std::endl;
			std::cout << std::string(52, '-') << std::endl;
			std::cout << std::setw(12) << "-"
					  << std::setw(15) << (uint32_t)__history->__history_bit_map
					  << std::endl;
		}
		else
		{
			std::cout << std::setw(12) << "Index"
					  << std::setw(15) << "History"
					  << std::endl;
			std::cout << std::string(52, '-') << std::endl;
			for (size_t i = 0; i < __btb_size; ++i)
			{
				std::cout << std::setw(12) << i
						  << std::setw(15) << (uint32_t)__history[i].__history_bit_map
						  << std::endl;
			}
		}
	}

	void printFSM() {

		std::cout << std::setw(12) << "   Index";
			std::cout << "  ";
		for (size_t i = 0; i < (1 << __history_size); ++i) {
			std::cout << " " << std::setw(3) << i;
		}
			std::cout << std::endl;

		std::cout << std::string(52, '-') << std::endl;

		if (is_global_table()) {
			// Global table: __fsm points to a single instance of Bimodal_FSM
			size_t table_size = (1 << __history_size); // Array length
			// std::cout << "Global FSM Table:\n";

			std::cout << std::setw(12) << "-" << "   ";
			for (size_t i = 0; i < table_size; ++i) {
				switch (__fsm->__bimodal_FSM_state[i]) {
					case SNT: std::cout << "SNT "; break;
					case WNT: std::cout << "WNT "; break;
					case WT:  std::cout << "WT "; break;
					case ST:  std::cout << "ST "; break;
				}
			}
			std::cout << "\n"; // End the line
		} else {
			// Per-entry table: __fsm points to an array of Bimodal_FSM
			for (size_t btb_index = 0; btb_index < __btb_size; ++btb_index) {
				size_t table_size = (1 << __history_size); // Array length
				//std::cout << "BTB Index " << btb_index << ":\n";

				std::cout << std::setw(12) << btb_index << "   ";
				for (size_t i = 0; i < table_size; ++i) {
					switch (__fsm[btb_index].__bimodal_FSM_state[i]) {
						case SNT: std::cout << "SNT "; break;
						case WNT: std::cout << "WNT "; break;
						case WT:  std::cout << "WT "; break;
						case ST:  std::cout << "ST "; break;
					}
				}
				std::cout << "\n"; // End the line
			}
		}
	}
		
	void print()
	{
		std::cout << std::endl;
		std::cout << "BTB table:" << std::endl;
		printBTBTable();
		std::cout << "History: " << std::endl;
		if (is_global_history())
		{
			std::cout << "global" << std::endl;
		}
		else
		{
			std::cout << "local" << std::endl;
		}
		printHistory();
		std::cout << "FSM: ";
		if (is_global_table())
		{
			std::cout << "global" << std::endl;
		}
		else
		{
			std::cout << "local" << std::endl;
		}
		printFSM();
		std::cout << std::endl;
	}
#endif
};

// ---*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-- //
// ? Global data

BranchPredictor *BRANCH_PREDICTOR;

// ---*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-- //
// ? Function implementation

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared)
{

	// * Create Branch Predictor
	BRANCH_PREDICTOR = new BranchPredictor(); // using the default c'tor
	// check if memory allocation is successful
	if (BRANCH_PREDICTOR == nullptr)
	{
		return -1;
	}

	// * Initialize init values
	// check if setting the value has failed
	if (BRANCH_PREDICTOR->set_btb_size(btbSize) == false)
	{
		delete BRANCH_PREDICTOR;
		return -1;
	}
	// check if setting the value has failed
	if (BRANCH_PREDICTOR->set_history_size(historySize) == false)
	{
		delete BRANCH_PREDICTOR;
		return -1;
	}
	// check if setting the value has failed
	if (BRANCH_PREDICTOR->set_tag_size(tagSize) == false)
	{
		delete BRANCH_PREDICTOR;
		return -1;
	}
	// check if setting the value has failed
	if (BRANCH_PREDICTOR->set_fsm_default_state(fsmState) == false)
	{
		delete BRANCH_PREDICTOR;
		return -1;
	}
	// check if setting the value has failed
	if (BRANCH_PREDICTOR->set_global_history(isGlobalHist) == false)
	{
		delete BRANCH_PREDICTOR;
		return -1;
	}
	// check if setting the value has failed
	if (BRANCH_PREDICTOR->set_global_table(isGlobalTable) == false)
	{
		delete BRANCH_PREDICTOR;
		return -1;
	}
	// check if setting the value has failed
	if (BRANCH_PREDICTOR->set_share(Shared) == false)
	{
		delete BRANCH_PREDICTOR;
		return -1;
	}

	// * Allocate tables for BTB, history, fsm table
	if (BRANCH_PREDICTOR->create_BTB() == false)
	{
		delete BRANCH_PREDICTOR;
		return -1;
	}
	if (BRANCH_PREDICTOR->create_history() == false)
	{
		delete BRANCH_PREDICTOR;
		return -1;
	}
	if (BRANCH_PREDICTOR->create_fsm_table() == false)
	{
		delete BRANCH_PREDICTOR;
		return -1;
	}

	// * Initialize stats
	BRANCH_PREDICTOR->Stats_flush_num() = 0;
	BRANCH_PREDICTOR->Stats_br_num() = 0;
	// Calculate the number of bits for the BTB
	size_t total_bits = 0;
	total_bits += (30 + BRANCH_PREDICTOR->get_tag_size() + 1) * (BRANCH_PREDICTOR->get_btb_size()); // (tag size + one valid bit + target pc bits without the padding 0's) * (BTB size)
	if (BRANCH_PREDICTOR->is_global_history())
	{
		total_bits += BRANCH_PREDICTOR->get_history_size();
	}
	else
	{
		total_bits += BRANCH_PREDICTOR->get_btb_size() * BRANCH_PREDICTOR->get_history_size();
	}
	// 2^(historySize) == (1 << historySize)
	if (BRANCH_PREDICTOR->is_global_table())
	{
		total_bits += 2 * (1 << BRANCH_PREDICTOR->get_history_size());
	}
	else
	{
		total_bits += BRANCH_PREDICTOR->get_btb_size() * 2 * (1 << BRANCH_PREDICTOR->get_history_size());
	}
	BRANCH_PREDICTOR->Stats_size() = total_bits;

#ifdef PRINT_AUX
	std::cout << "Init:" << std::endl; 
	BRANCH_PREDICTOR->print();
#endif

	return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst)
{

#ifdef PRINT_AUX
	std::cout << std::endl
			  << "before predict" << std::endl;
	BRANCH_PREDICTOR->print();
#endif

	// taken == true, not taken == false

	// * calculate BTB entry index
	int index = get_index(pc, (uint8_t)std::log2(BRANCH_PREDICTOR->get_btb_size()));
	// ** check if tags are the same for
	uint32_t pc_tag = get_tag(pc, BRANCH_PREDICTOR->get_tag_size(), (uint8_t)std::log2(BRANCH_PREDICTOR->get_btb_size()));
	if (BRANCH_PREDICTOR->is_BTB_entry_valid(index))
	{
		uint32_t prediction_pc = BRANCH_PREDICTOR->get_BTB_entry_branch_pc(index);
		uint32_t prediction_tag = get_tag(prediction_pc, BRANCH_PREDICTOR->get_tag_size(), (uint8_t)std::log2(BRANCH_PREDICTOR->get_btb_size()), 2);
		if (pc_tag == prediction_tag) // same entry in BTB => possibly the same instruction
		{
			// return prediction
			bool taken = BRANCH_PREDICTOR->get_BTB_prediction(pc);
			if (taken)
			{
				*dst = BRANCH_PREDICTOR->get_BTB_entry_target_pc(index);
				return true; // taken
			} // else => no taken == false
		}
		else
		{
			BRANCH_PREDICTOR->set_BTB_as_invalid(index);
		}
	}
	*dst = pc + 4; // Next sequential address

#ifdef PRINT_AUX
	std::cout << std::endl
			  << "after predict" << std::endl;
	BRANCH_PREDICTOR->print();
#endif
	return false; // not taken
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst)
{

#ifdef PRINT_AUX
	std::cout << std::endl
			  << "before update" << std::endl;
	BRANCH_PREDICTOR->print();
#endif

	BRANCH_PREDICTOR->update(pc, targetPc, taken, pred_dst);

#ifdef PRINT_AUX
	std::cout << std::endl
			  << "after update" << std::endl;
	BRANCH_PREDICTOR->print();
#endif
}

void BP_GetStats(SIM_stats *curStats)
{
	*curStats = BRANCH_PREDICTOR->get_stats();
	delete BRANCH_PREDICTOR;
	return;
}

// ---*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-----*-- //
// ? Aux Functions and Classes Function implementation

uint32_t get_tag(uint32_t pc, uint8_t tag_length, uint8_t index_length, uint8_t alignment)
{
	// ((1 << tag_length) - 1) is 0..01..1 where # of 1's is `tag_length`
	return (pc >> (alignment + index_length)) & ((1 << tag_length) - 1);
}

uint32_t get_index(uint32_t pc, uint8_t index_length, uint8_t alignment)
{
	/**
	 * (1 << index_length) gives a number of the form: 0...010...0 where # of LSB zeros is
	 * 	the index_length, removing 1 gives replaces the 1 by 0 and replaces the
	 * 	[index_length] LSB 0 bits to 1's, doing an & with this gives the [index_length] LSB
	 * 	bits of (pc >> alignment) where this is the (pc) without the padding 0's got from
	 * 	the alignment of the pc (if any).
	 */
	return (pc >> alignment) & ((1 << index_length) - 1);
}

BranchPredictor::~BranchPredictor()
{
	delete[] __btb;

	delete[] __history;

	if (__fsm)
	{
		if (this->is_global_table())
		{
			delete[] __fsm->__bimodal_FSM_state;
		}
		else
		{
			for (size_t i = 0; i < this->get_btb_size(); i++)
			{
				delete[] __fsm[i].__bimodal_FSM_state;
			}
		}
	}
	delete[] __fsm;
}

bool BranchPredictor::create_BTB()
{
	__btb = new BTB_entry[this->get_btb_size()];
	if (__btb == nullptr)
	{
		return false;
	}
	return true;
}

bool BranchPredictor::create_history()
{
	if (this->is_global_history())
	{
		// make only 1 history
		__history = new BTB_history[1];
	}
	else
	{
		// make |BTB| histories, one for each BTB entry
		__history = new BTB_history[this->get_btb_size()];
	}

	if (__history == nullptr)
	{
		return false;
	}
	return true;
}

bool BranchPredictor::create_fsm_table()
{
	if (this->is_global_table())
	{
		// make only 1 fsm table
		__fsm = new Bimodal_FSM[1];
		if (__fsm == nullptr)
		{
			return false;
		}
		__fsm->__bimodal_FSM_state = new fsm_state[1 << this->get_history_size()];
		if (__fsm->__bimodal_FSM_state == nullptr)
		{
			delete[] __fsm;
			__fsm = nullptr;
			__fsm = nullptr;
			return false;
		}
		// initialize it
		for (size_t j = 0; j < (1 << this->get_history_size()); j++)
		{
			(__fsm->__bimodal_FSM_state)[j] = (fsm_state)this->get_fsm_default_state();
		}
	}
	else
	{
		// make |BTB| fsm table(s), one for each BTB entry
		__fsm = new Bimodal_FSM[this->get_btb_size()];
		if (__fsm == nullptr)
		{
			return false;
		}
		for (size_t i = 0; i < this->get_btb_size(); i++)
		{
			__fsm[i].__bimodal_FSM_state = new fsm_state[1 << this->get_history_size()];
			if (__fsm[i].__bimodal_FSM_state == nullptr)
			{
				for (size_t j = 0; j < i; j++)
				{
					delete[] __fsm[j].__bimodal_FSM_state;
					__fsm[j].__bimodal_FSM_state = nullptr;
				}
				delete[] __fsm;
				__fsm = nullptr;
				return false;
			}
			// initialize it
			for (size_t j = 0; j < (1 << this->get_history_size()); j++)
			{
				(__fsm[i].__bimodal_FSM_state)[j] = (fsm_state)this->get_fsm_default_state();
			}
		}
	}
	return true;
}

bool BranchPredictor::get_BTB_prediction(uint32_t pc)
{
	// taken == true, not taken == false

	// get history == fsm index
	uint32_t fsm_index = this->get_fsm_index(pc);
	uint32_t index = get_index(pc, (uint8_t)std::log2(this->get_btb_size()));

	uint32_t tag_branch = get_tag(this->get_BTB_entry_branch_pc(index), this->get_tag_size(), (uint8_t)std::log2(this->get_btb_size()));
	uint32_t tag_pc = get_tag(pc, this->get_tag_size(), (uint8_t)std::log2(this->get_btb_size()));
	if (this->is_BTB_entry_valid(index) && (tag_branch == tag_pc))
	{
		if (BRANCH_PREDICTOR->is_global_table())
		{
			// remember, __fsm is global
			return (__fsm->__bimodal_FSM_state[fsm_index] >= 2);
		}
		else // local table => __share doesn't matter
		{
			// remember, __fsm is local
			return (__fsm[index].__bimodal_FSM_state[fsm_index] >= 2);
		}
	}
	return false;
}

uint32_t BranchPredictor::get_fsm_index(uint32_t pc)
{
	uint32_t index = get_index(pc, (uint8_t)std::log2(this->get_btb_size()));
	// get history == fsm index
	uint32_t fsm_index;
	if (this->is_global_history())
	{
		fsm_index = __history->__history_bit_map;
	}
	else // local history
	{
		fsm_index = __history[index].__history_bit_map;
	}

	if (BRANCH_PREDICTOR->is_global_table())
	{
		if (BRANCH_PREDICTOR->get_share() == 1) // using_share_lsb == 1 : XOR history with bit 2 -> MSB
		{
			fsm_index ^= ((pc >> 2) & ((1 << this->get_history_size()) - 1));
		}
		else if (BRANCH_PREDICTOR->get_share() == 2) // using_share_mid == 2 : XOR history with bit 16 -> MSB
		{
			fsm_index ^= ((pc >> 16) & ((1 << this->get_history_size()) - 1));
		}
	}
	return fsm_index;
}

void BranchPredictor::update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst)
{

	// * update the stats
	// dealing with another branch instruction
	this->Stats_br_num()++;
	// mis-predictions causes a flush

	uint32_t branch_index = get_index(pc, (uint8_t)std::log2(this->get_btb_size()));
	uint32_t prediction = BRANCH_PREDICTOR->get_BTB_prediction(pc);
	if (((prediction == taken) && (taken && (pred_dst != targetPc))) ||
		((prediction == taken) && (!taken && (pred_dst != pc + 4))) ||
		prediction != taken)
	{
		this->Stats_flush_num()++;
	}

	// * update the BTB
	// check if pc is already in BTB, if not our prediction is not taken
	// uint32_t btb_branch_pc = this->get_BTB_entry_branch_pc(branch_index);
	// uint32_t branch_tag = get_tag(pc, this->get_tag_size(), (uint8_t)std::log2(this->get_btb_size()), 2);
	// uint32_t btb_branch_tag = get_tag(btb_branch_pc, this->get_tag_size(), (uint8_t)std::log2(this->get_btb_size()), 2);

	this->set_BTB_entry_target_pc(branch_index, targetPc);

	if (!this->is_BTB_entry_valid(branch_index))
	{
		// mark the entry as valid
		this->set_BTB_as_valid(branch_index);
		// change the entry in the BTB for the new branch
		this->set_BTB_entry_branch_pc(branch_index, pc);

		if (!this->is_global_history()) // local history
		{
			__history[branch_index].__history_bit_map = 0;
		}

		if (!this->is_global_table()) // local fsm
		{
			for (size_t i = 0; i < (1 << this->get_history_size()); i++)
			{
				__fsm[branch_index].__bimodal_FSM_state[i] = this->__fsm_default_state;
			}

			/*
			if (this->is_global_history())
			{
				uint32_t fsm_index = branch_index;
				if (this->get_share() == 1) // using_share_lsb == 1 : XOR history with bit 2 -> MSB
				{
					fsm_index ^= ((this->get_BTB_entry_branch_pc(branch_index) >> 2) & ((1 << this->get_history_size()) - 1));
				}
				else if (this->get_share() == 2) // using_share_mid == 2 : XOR history with bit 16 -> MSB
				{
					fsm_index ^= ((this->get_BTB_entry_branch_pc(branch_index) >> 16) & ((1 << this->get_history_size()) - 1));
				}
				// else don't change index using XOR

				__fsm[branch_index].__bimodal_FSM_state[fsm_index] = this->__fsm_default_state;
			}
			else
			{
				__fsm[branch_index].__bimodal_FSM_state[__history[branch_index].__history_bit_map] = this->__fsm_default_state;
			}
			*/
		}
	}

	uint32_t fsm_index = this->get_fsm_index(pc);
	if (this->is_global_table())
	{
		fsm_state &s = __fsm->__bimodal_FSM_state[fsm_index];
		this->updateFSMState(s, taken);
	}
	else
	{
		fsm_state &s = __fsm[branch_index].__bimodal_FSM_state[fsm_index];
		this->updateFSMState(s, taken);
	}

	if (this->is_global_history())
	{
		uint8_t &history = __history->__history_bit_map;
		history = history << 1;
		history &= (1 << this->get_history_size()) - 1;
		if (taken)
		{
			history++;
		}
	}
	else
	{
		uint8_t &history = __history[branch_index].__history_bit_map;
		history = history << 1;
		history &= (1 << this->get_history_size()) - 1;
		if (taken)
		{
			history++;
		}
	}

	/*
			// change the entry in the BTB for the new branch
			this->set_BTB_entry_branch_pc(branch_index, pc);
			// re-initialize the history and fsm
			if (!this->is_global_history()) // local history
			{
				__history[branch_index].__history_bit_map = 0;
			}
			else
			{
				__history->__history_bit_map = (__history->__history_bit_map << 1) & ((1 << this->get_history_size()) - 1);
				if (taken)
				{
					__history->__history_bit_map++;
				}
			}

			if (!this->is_global_table()) // local fsm
			{
				if (this->is_global_history())
				{
					uint32_t fsm_index = branch_index;
					if (this->get_share() == 1) // using_share_lsb == 1 : XOR history with bit 2 -> MSB
					{
						fsm_index ^= ((pc >> 2) & ((1 << this->get_history_size()) - 1));
					}
					else if (this->get_share() == 2) // using_share_mid == 2 : XOR history with bit 16 -> MSB
					{
						fsm_index ^= ((pc >> 16) & ((1 << this->get_history_size()) - 1));
					}
					// else don't change index using XOR

					__fsm[branch_index].__bimodal_FSM_state[fsm_index] = this->__fsm_default_state;
				}
				else // local history
				{
					__fsm[branch_index].__bimodal_FSM_state[__history[branch_index].__history_bit_map] = this->__fsm_default_state;
				}
			}
	*/

	/*
		// if they have the same tag => potentially the same instruction
		if (branch_tag == btb_branch_tag)
		{
			// there can be cases where we can'y say for sure (be decisive) whether the branches in the BTB index are the same
			// if BTB is decisive we change the entry, if not we only update it the first time
			if (this->is_BTB_decisive())
			{
			}
			// else no init. is done
		}
		else // they are NOT the same instruction
		{
		}
	*/

	return;
}
